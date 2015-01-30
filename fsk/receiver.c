#include <assert.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdbool.h>
#include <math.h>

#include "pa_ringbuffer.h"
#include <portaudio.h>

#define RING_BUFFER_SIZE (1 << 12)

#define SAMPLE_RATE 44100
#define FRAMES_PER_BUFFER paFramesPerBufferUnspecified

#define SAMPLE_SILENCE 0.f

#define BAUD 10
#define ZERO_FREQ 1200.f
#define ONE_FREQ 2200.f

struct callback_data {
	PaUtilRingBuffer *ring_buffer;
	int phase;
	int frame;
	int bit_index;
	char byte;
	bool bit;
};

static int receive_callback(const void *input_buffer, void *output_buffer,
			 unsigned long frames_per_buffer,
			 const PaStreamCallbackTimeInfo *time_info,
                            PaStreamCallbackFlags status_flags, void *arg)
{

        struct callback_data *data = arg;

	ring_buffer_size_t elements_writeable =
		PaUtil_GetRingBufferWriteAvailable(data->ring_buffer);
	ring_buffer_size_t elements_to_write =
		fmin(elements_writeable, (ring_buffer_size_t)frames_per_buffer);
	const float *in = (const float *) input_buffer;

	(void)output_buffer;
	(void)frames_per_buffer;
	(void)time_info;
	(void)status_flags;

	unsigned long i;
	if (in == NULL) {
		float silence = SAMPLE_SILENCE;
		for (i = 0; i < frames_per_buffer; i++) {
			PaUtil_WriteRingBuffer(data->ring_buffer, &silence, 1);
		}
	} else {
		PaUtil_WriteRingBuffer(data->ring_buffer, in, elements_to_write);
	}

        return paContinue;
}

int main(void)
{
	PaUtilRingBuffer ring_buffer;
	ring_buffer_size_t ring_ret;
	void *ring_buffer_ptr;
	PaStream *stream;
	PaError err;

	struct callback_data data = {
		.ring_buffer = &ring_buffer,
		.frame = SAMPLE_RATE / BAUD,
		.bit_index = 8,
	};

	int status = EXIT_SUCCESS;
	
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
				   FRAMES_PER_BUFFER, receive_callback, &data);
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

	float x = 0.f;
	while (true) {
		ring_ret = PaUtil_ReadRingBuffer(data.ring_buffer, &x, 1);
		if (ring_ret > 0) {
			printf("%f\n", x);
		}
		Pa_Sleep(1000);
	}

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

