#include <assert.h>
#include <complex.h>
#include <math.h>
#include <signal.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "fftw3.h"
#include "portaudio.h"
#include "pa_ringbuffer.h"

#define RING_BUFFER_SIZE (1 << 12)

#define SAMPLE_RATE 44100
#define FRAMES_PER_BUFFER paFramesPerBufferUnspecified

#define FFT_WINDOW 256
#define SLIDE_WINDOW (FFT_WINDOW / 4)

#define ZERO_FREQ 2200.f
#define ONE_FREQ 1200.f

static int receive_callback(const void *input_buffer, void *output_buffer,
			    unsigned long frames_per_buffer,
			    const PaStreamCallbackTimeInfo *time_info,
			    PaStreamCallbackFlags status_flags, void *arg)
{
	(void)output_buffer;
	(void)time_info;
	(void)status_flags;

	PaUtil_WriteRingBuffer(arg, input_buffer, frames_per_buffer);

        return paContinue;
}

static sig_atomic_t signal_received;
static void signal_handler(int signum)
{
	signal_received = signum;
}

/* Convert window to frame to time in seconds. */
static inline float window_to_seconds(int window)
{
	return (float)window * (float)FFT_WINDOW / (float)SAMPLE_RATE;
}

/* Convert FFT bin index to frequency in Hz. */
static inline float bin_to_frequency(int bin)
{
	return (float)bin * (float)SAMPLE_RATE / (float)FFT_WINDOW;
}

/* Convert FFT output to dBFS. */
static inline float fft_to_dbfs(float complex fft)
{
	return 20.f * logf(2.f * cabsf(fft) / (float)FFT_WINDOW);
}

void receiver_loop(PaUtilRingBuffer *ring_buffer, const fftwf_plan fft_plan,
		   float *fft_in, float complex *fft_out)
{
	ring_buffer_size_t ring_ret;
	int signum;
	int window = 0;

	while (!(signum = signal_received)) {
		float time, frequency, dbfs;

		if (PaUtil_GetRingBufferReadAvailable(ring_buffer) < FFT_WINDOW / 2)
			continue;

		memmove(fft_in, fft_in + FFT_WINDOW / 2,
			(FFT_WINDOW / 2) * sizeof(float));
		ring_ret = PaUtil_ReadRingBuffer(ring_buffer, fft_in + FFT_WINDOW / 2,
						 FFT_WINDOW / 2);
		assert(ring_ret == FFT_WINDOW / 2);

		fftwf_execute(fft_plan);

		time = window_to_seconds(window);
		for (int i = 0; i < FFT_WINDOW / 2 + 1; i++) {
			frequency = bin_to_frequency(i);
			dbfs = fft_to_dbfs(fft_out[i]);
			printf("%f %f %f\n", time, frequency, dbfs);
		}
		printf("\n");

		window++;
	}

	fprintf(stderr, "got %s; exiting\n", strsignal(signum));
}

int main(void)
{
	fftwf_plan fft_plan = NULL;
	float *fft_in = NULL;
	float complex *fft_out = NULL;

	PaUtilRingBuffer ring_buffer;
	void *ring_buffer_ptr;

	PaStream *stream;
	PaError err;

	int status = EXIT_SUCCESS;
	struct sigaction sa;

	/* Handle signals. */
	sa.sa_handler = signal_handler;
	sa.sa_flags = SA_RESTART;
	sigemptyset(&sa.sa_mask);
	if (sigaction(SIGINT, &sa, NULL) == -1) {
		perror("sigaction");
		return EXIT_FAILURE;
	}

	/* Initialize ring buffer. */
	ring_buffer_ptr = malloc(RING_BUFFER_SIZE * sizeof(float));
	if (!ring_buffer_ptr) {
		perror("malloc");
		return EXIT_FAILURE;
	}
	PaUtil_InitializeRingBuffer(&ring_buffer, sizeof(float), RING_BUFFER_SIZE,
				    ring_buffer_ptr);

	/* Initialize FFTW. */
	fft_in = fftwf_alloc_real(FFT_WINDOW);
	if (!fft_in) {
		fprintf(stderr, "could not allocate FFT input\n");
		status = EXIT_FAILURE;
		goto out;
	}
	memset(fft_in, 0, FFT_WINDOW * sizeof(float));
	fft_out = fftwf_alloc_complex(FFT_WINDOW / 2 + 1);
	if (!fft_out) {
		fprintf(stderr, "could not allocate FFT output\n");
		status = EXIT_FAILURE;
		goto fftw_cleanup;
	}
	fft_plan = fftwf_plan_dft_r2c_1d(FFT_WINDOW, fft_in, fft_out, 0);
	if (!fft_plan) {
		fprintf(stderr, "could not create FFT plan\n");
		status = EXIT_FAILURE;
		goto fftw_cleanup;
	}

	/* Initialize PortAudio. */
	err = Pa_Initialize();
	if (err != paNoError) {
		fprintf(stderr, "PortAudio: initialization failed: %s\n",
			Pa_GetErrorText(err));
		status = EXIT_FAILURE;
		goto fftw_cleanup;
	}

	err = Pa_OpenDefaultStream(&stream, 1, 0, paFloat32, SAMPLE_RATE,
				   FRAMES_PER_BUFFER, receive_callback,
				   &ring_buffer);
	if (err != paNoError) {
		fprintf(stderr, "PortAudio: opening stream failed: %s\n",
			Pa_GetErrorText(err));
		status = EXIT_FAILURE;
		goto terminate;
	}

	/* Run the receiver. */
	err = Pa_StartStream(stream);
	if (err != paNoError) {
		fprintf(stderr, "PortAudio: starting stream failed: %s\n",
			Pa_GetErrorText(err));
		status = EXIT_FAILURE;
		goto close_stream;
	}

	receiver_loop(&ring_buffer, fft_plan, fft_in, fft_out);

	err = Pa_StopStream(stream);
	if (err != paNoError) {
		fprintf(stderr, "PortAudio: stopping stream failed: %s\n",
			Pa_GetErrorText(err));
		status = EXIT_FAILURE;
		goto close_stream;
	}

	/* Cleanup. */
close_stream:
	err = Pa_CloseStream(stream);
	if (err != paNoError) {
		fprintf(stderr, "PortAudio: closing stream failed: %s\n",
			Pa_GetErrorText(err));
		status = EXIT_FAILURE;
	}

terminate:
	err = Pa_Terminate();
	if (err != paNoError) {
		fprintf(stderr, "PortAudio: termination failed: %s\n",
			Pa_GetErrorText(err));
		status = EXIT_FAILURE;
	}

fftw_cleanup:
	if (fft_plan)
		fftwf_destroy_plan(fft_plan);
	fftwf_free(fft_in);
	fftwf_free(fft_out);
	fftwf_cleanup();

out:
	free(ring_buffer_ptr);
	return status;
}

