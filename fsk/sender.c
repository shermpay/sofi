#include <assert.h>
#include <math.h>
#include <portaudio.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "pa_ringbuffer.h"

#define RING_BUFFER_SIZE (1 << 12)

#define SAMPLE_RATE 44100
#define FRAMES_PER_BUFFER paFramesPerBufferUnspecified

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

static int send_callback(const void *input_buffer, void *output_buffer,
			 unsigned long frames_per_buffer,
			 const PaStreamCallbackTimeInfo *time_info,
			 PaStreamCallbackFlags status_flags, void *arg)
{
	float *out = output_buffer;
	struct callback_data *data = arg;
	float frequency;
	ring_buffer_size_t ret;
	unsigned long i;

	(void)input_buffer;
	(void)time_info;
	(void)status_flags;

	for (i = 0; i < frames_per_buffer; i++) {
		if (data->frame >= SAMPLE_RATE / BAUD) {
			if (data->bit_index >= 8) {
				ret = PaUtil_ReadRingBuffer(data->ring_buffer,
							    &data->byte, 1);
				if (ret == 0)
					break;
				data->bit_index = 0;
			}
			data->frame = 0;
			data->bit = data->byte & (1 << data->bit_index);
			data->bit_index++;
		}
		data->frame++;

		frequency = data->bit ? ONE_FREQ : ZERO_FREQ;

		out[i] = sinf(2 * M_PI * frequency * data->phase / SAMPLE_RATE);
		data->phase++;
	}

	for (; i < frames_per_buffer; i++)
		out[i] = 0.f;

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
	char *line = NULL;
	size_t n = 0;
	ssize_t ret;
	int status = EXIT_SUCCESS;

	ring_buffer_ptr = malloc(RING_BUFFER_SIZE);
	if (!ring_buffer_ptr) {
		perror("malloc");
		return EXIT_FAILURE;
	}

	PaUtil_InitializeRingBuffer(&ring_buffer, 1, RING_BUFFER_SIZE,
				    ring_buffer_ptr);

	err = Pa_Initialize();
	if (err != paNoError) {
		fprintf(stderr, "PortAudio: initialization failed: %s\n",
			Pa_GetErrorText(err));
		status = EXIT_FAILURE;
		goto out;
	}

	err = Pa_OpenDefaultStream(&stream, 0, 1, paFloat32, SAMPLE_RATE,
				   FRAMES_PER_BUFFER, send_callback, &data);
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

	while ((ret = getline(&line, &n, stdin)) > 0) {
		ring_ret = PaUtil_WriteRingBuffer(&ring_buffer, line,
						  (ring_buffer_size_t)ret);
		assert(ring_ret == (ring_buffer_size_t)ret);
	}

	if (ret == -1 && !feof(stdin)) {
		perror("getline");
		status = EXIT_FAILURE;
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
	free(line);
	free(ring_buffer_ptr);
	return status;
}
