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
#include "fsk.h"

#define RING_BUFFER_SIZE (1 << 12)

#define SAMPLE_RATE 44100
#define FRAMES_PER_BUFFER 1024

#define FFT_WINDOW 1024
#define ACTUAL_WINDOW 256
#define SLIDE_WINDOW (ACTUAL_WINDOW / 4)

#define DELTA_STEADY (1.f / (32.f * BAUD))
#define DEMOD_WINDOW (1.f / (2.f * BAUD))

static int receive_callback(const void *input_buffer, void *output_buffer,
			    unsigned long frames_per_buffer,
			    const PaStreamCallbackTimeInfo *time_info,
			    PaStreamCallbackFlags status_flags, void *arg)
{
	ring_buffer_size_t ret;
	(void)output_buffer;
	(void)time_info;
	(void)status_flags;

	assert((unsigned long)PaUtil_GetRingBufferWriteAvailable(arg) >= frames_per_buffer);

	ret = PaUtil_WriteRingBuffer(arg, input_buffer, frames_per_buffer);
	assert((unsigned long)ret == frames_per_buffer);

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
	return (float)window * (float)SLIDE_WINDOW / (float)SAMPLE_RATE;
}

/* Convert FFT bin index to frequency in Hz. */
static inline float bin_to_frequency(int bin)
{
	return (float)bin * (float)SAMPLE_RATE / (float)FFT_WINDOW;
}

/* Convert frequency in Hz to FFT bin. */
static inline int frequency_to_bin(float hz)
{
	return (int)(hz * (float)FFT_WINDOW / (float)SAMPLE_RATE + 0.5f);
}

/* Convert FFT output to dBFS. */
static inline float fft_to_dbfs(float complex fft)
{
	return 20.f * logf(2.f * cabsf(fft) / (float)FFT_WINDOW);
}

enum state {
	STATE_LISTENING,
	STATE_NOISE_WAIT,
	STATE_DEMOD_WAIT,
	STATE_DEMOD_GATHER,
};

struct sig_counts {
	int one;
	int zero;
	int none;
};

#define MOSTLY_THRESHOLD 0.50f

static inline int calc_mostly(struct sig_counts *counts)
{
	int total = counts->one + counts->zero + counts->none;

	if ((float)counts->one / (float)total >= MOSTLY_THRESHOLD)
		return 1;
	else if ((float)counts->zero / (float)total >= MOSTLY_THRESHOLD)
		return 0;
	else
		return -1;
}

void receiver_loop(PaUtilRingBuffer *ring_buffer, const fftwf_plan fft_plan,
		   float *fft_in, float complex *fft_out)
{
	ring_buffer_size_t ring_ret;
	int signum;
	int window = 0;
	enum state state = STATE_LISTENING;
	float t0;
	float wait_until;
	int prev = 0;
	int n;
	struct sig_counts counts;

	while (!(signum = signal_received)) {
		float time;
		float zero_dbfs, one_dbfs;
		int val;

		if (PaUtil_GetRingBufferReadAvailable(ring_buffer) < SLIDE_WINDOW)
			continue;

		/* Compute the FFT. */
		memmove(fft_in, fft_in + SLIDE_WINDOW,
			(ACTUAL_WINDOW - SLIDE_WINDOW) * sizeof(float));
		ring_ret = PaUtil_ReadRingBuffer(ring_buffer,
						 fft_in + (ACTUAL_WINDOW - SLIDE_WINDOW),
						 SLIDE_WINDOW);
		assert(ring_ret == SLIDE_WINDOW);

		fftwf_execute(fft_plan);

		/* Compute the current value in the raw stream. */
		time = window_to_seconds(window++);

		zero_dbfs = fft_to_dbfs(fft_out[frequency_to_bin(ZERO_FREQ)]);
		one_dbfs = fft_to_dbfs(fft_out[frequency_to_bin(ONE_FREQ)]);

		if ((zero_dbfs < -75.f && one_dbfs < -75.f) ||
		    (fabs(zero_dbfs - one_dbfs) < 5.f))
			val = 0;
		else if (zero_dbfs > one_dbfs)
			val = -1;
		else
			val = 1;

		switch (state) {
		case STATE_LISTENING:
			if (val != prev) {
				t0 = time;
				wait_until = t0 + DELTA_STEADY;
				state = STATE_NOISE_WAIT;
			}
			prev = val;
			break;
		case STATE_NOISE_WAIT:
			if (time >= wait_until) {
				n = 0;
				wait_until = t0 + (1.f / (2.f * BAUD)) + (n / (float)BAUD) - (DEMOD_WINDOW / 2.f);
				state = STATE_DEMOD_WAIT;
			} else {
				if (val != prev) {
					state = STATE_LISTENING;
				}
				prev = val;
			}
			break;
		case STATE_DEMOD_WAIT:
			if (time >= wait_until) {
				counts.zero = 0;
				counts.one = 0;
				counts.none = 0;
				wait_until = t0 + (1.f / (2.f * BAUD)) + (n / (float)BAUD) + (DEMOD_WINDOW / 2.f);
				state = STATE_DEMOD_GATHER;
			}
			break;
		case STATE_DEMOD_GATHER:
			if (time >= wait_until) {
				int mostly;

				mostly = calc_mostly(&counts);
				if (mostly == 0) {
					printf("%d\n", 0);
				} else if (mostly == 1) {
					printf("%d\n", 1);
				} else {
					state = STATE_LISTENING;
					break;
				}

				n++;
				wait_until = t0 + (1.f / (2.f * BAUD)) + (n / (float)BAUD) - (DEMOD_WINDOW / 2.f);
				state = STATE_DEMOD_WAIT;
			} else {
				fprintf(stderr, "%f 0\n", time);

				if (val == -1)
					counts.zero++;
				else if (val == 1)
					counts.one++;
				else
					counts.none++;
			}
			break;
		default:
			break;
		}
	}

	/* fprintf(stderr, "got %s; exiting\n", strsignal(signum)); */
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

