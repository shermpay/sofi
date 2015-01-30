#include <assert.h>
#include <math.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "portaudio.h"

#define RING_BUFFER_SIZE (1 << 12)

#define SAMPLE_RATE 44100
#define FRAMES_PER_BUFFER paFramesPerBufferUnspecified

#define BAUD 10
#define ZERO_FREQ 1200.f
#define ONE_FREQ 2200.f
#define ZERO_AMP 1.f
#define ONE_AMP 1.f

struct callback_data {
	char *data;
	size_t len;
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
	float amp;
	unsigned long i;

	(void)input_buffer;
	(void)time_info;
	(void)status_flags;

	for (i = 0; i < frames_per_buffer; i++) {
		if (++data->frame >= SAMPLE_RATE / BAUD) {
			if (++data->bit_index >= 8) {
				if (data->len == 0)
					break;
				data->byte = *data->data++;
				data->len--;
				data->bit_index = 0;
			}
			data->bit = data->byte & (1 << data->bit_index);
			data->frame = 0;
		}

		frequency = data->bit ? ONE_FREQ : ZERO_FREQ;
		amp = data->bit ? ONE_AMP : ZERO_AMP;

		out[i] = amp * sinf(2 * M_PI * frequency * data->phase / SAMPLE_RATE);
		data->phase++;
	}

	if (i < frames_per_buffer) {
		for (; i < frames_per_buffer; i++)
			out[i] = 0.f;
		return paComplete;
	}

	return paContinue;
}

int main(void)
{
	PaStream *stream;
	PaError err;
	struct callback_data data;
	char *line = NULL;
	size_t n = 0;
	ssize_t ret;
	int status = EXIT_SUCCESS;

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

	while ((ret = getline(&line, &n, stdin)) > 0) {
		data.data = line;
		data.len = ret;
		data.phase = 0;
		data.frame = SAMPLE_RATE / BAUD - 1;
		data.bit_index = 7;

		err = Pa_StartStream(stream);
		if (err != paNoError) {
			fprintf(stderr, "PortAudio: starting stream failed: %s\n",
				Pa_GetErrorText(err));
			status = EXIT_FAILURE;
			goto close_stream;
		}

		while ((err = Pa_IsStreamActive(stream)) == 1)
			Pa_Sleep(100); /* XXX: gross. */

		if (err != paNoError) {
			fprintf(stderr, "PortAudio: checking stream failed: %s\n",
				Pa_GetErrorText(err));
			status = EXIT_FAILURE;
		}


		err = Pa_StopStream(stream);
		if (err != paNoError) {
			fprintf(stderr, "PortAudio: stopping stream failed: %s\n",
				Pa_GetErrorText(err));
			status = EXIT_FAILURE;
			goto close_stream;
		}
	}

	if (ret == -1 && !feof(stdin)) {
		perror("getline");
		status = EXIT_FAILURE;
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
	return status;
}
