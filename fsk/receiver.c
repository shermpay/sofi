#include <assert.h>
#include <math.h>
#include <signal.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "portaudio.h"
#include "pa_ringbuffer.h"

#define RING_BUFFER_SIZE (1 << 12)

#define SAMPLE_RATE 44100
#define FRAMES_PER_BUFFER paFramesPerBufferUnspecified

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

void receiver_loop(PaUtilRingBuffer *ring_buffer)
{
	ring_buffer_size_t ring_ret;
	float x;
	int signum;

	while (!(signum = signal_received)) {
		ring_ret = PaUtil_ReadRingBuffer(ring_buffer, &x, 1);
		if (ring_ret > 0)
			printf("%f\n", x);
	}

	fprintf(stderr, "got %s; exiting\n", strsignal(signum));
}

int main(void)
{
	PaUtilRingBuffer ring_buffer;
	void *ring_buffer_ptr;
	PaStream *stream;
	PaError err;
	int status = EXIT_SUCCESS;
	struct sigaction sa;

	sa.sa_handler = signal_handler;
	sa.sa_flags = SA_RESTART;
	sigemptyset(&sa.sa_mask);
	if (sigaction(SIGINT, &sa, NULL) == -1) {
		perror("sigaction");
		return EXIT_FAILURE;
	}

	ring_buffer_ptr = malloc(RING_BUFFER_SIZE * sizeof(float));
	if (!ring_buffer_ptr) {
		perror("malloc");
		return EXIT_FAILURE;
	}

	PaUtil_InitializeRingBuffer(&ring_buffer, sizeof(float), RING_BUFFER_SIZE,
				    ring_buffer_ptr);

	err = Pa_Initialize();
	if (err != paNoError) {
		fprintf(stderr, "PortAudio: initialization failed: %s\n",
			Pa_GetErrorText(err));
		status = EXIT_FAILURE;
		goto out;
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

	err = Pa_StartStream(stream);
	if (err != paNoError) {
		fprintf(stderr, "PortAudio: starting stream failed: %s\n",
			Pa_GetErrorText(err));
		status = EXIT_FAILURE;
		goto close_stream;
	}

	receiver_loop(&ring_buffer);

	err = Pa_StopStream(stream);
	if (err != paNoError) {
		fprintf(stderr, "PortAudio: stopping stream failed: %s\n",
			Pa_GetErrorText(err));
		status = EXIT_FAILURE;
		goto close_stream;
	}

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

out:
	free(ring_buffer_ptr);
	return status;
}

