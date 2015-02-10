#include <assert.h>
#include <math.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "portaudio.h"
#include "pa_ringbuffer.h"
#include "fsk.h"

#define RING_BUFFER_SIZE (1 << 12)

#define SAMPLE_RATE 44100
#define FRAMES_PER_BUFFER paFramesPerBufferUnspecified

#define ZERO_AMP 1.f
#define ONE_AMP 1.f

enum len_state {
        LSTATE_READING,
        LSTATE_SENDING,
        LSTATE_DONE,
};

struct callback_data {
	PaUtilRingBuffer *ring_buffer;
	bool first;
        enum len_state lstate;
	float phase;
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
	ring_buffer_size_t ring_ret;
	float *out = output_buffer;
	struct callback_data *data = arg;
	float frequency;
	float amp;
	unsigned long i;
	void *data1, *data2;
	ring_buffer_size_t size1, size2;

	(void)input_buffer;
	(void)time_info;
	(void)status_flags;

	for (i = 0; i < frames_per_buffer; i++) {
		if (data->first || data->frame + 1 >= SAMPLE_RATE / BAUD) {
			if ((data->lstate != LSTATE_SENDING && data->first) || data->bit_index + 1 >= 8) {
                                if (data->lstate == LSTATE_SENDING) {
                                        data->lstate = LSTATE_DONE;
                                        data->frame++;
                                }
                                if (data->lstate == LSTATE_READING) {
                                        ring_ret = PaUtil_GetRingBufferReadAvailable(data->ring_buffer);
                                        if (ring_ret == 0)
                                                break;
                                        data->lstate = LSTATE_SENDING;
                                        data->byte = ring_ret;
                                } else {
                                        if (!data->first)
                                                PaUtil_AdvanceRingBufferReadIndex(data->ring_buffer, 1);
                                        else
                                                data->first = false;
                                        ring_ret = PaUtil_GetRingBufferReadRegions(data->ring_buffer, 1,
                                                                                   &data1, &size1,
                                                                                   &data2, &size2);
                                        if (ring_ret == 0)
                                                break;
                                        assert(size1 == 1);
                                        assert(size2 == 0);
                                        data->byte = *(char *)data1;
                                }
				data->bit_index = 0;
			} else {
				data->bit_index++;
			}
			data->bit = data->byte & (1 << data->bit_index);
			data->frame = 0;
		} else {
			data->frame++;
		}

		frequency = data->bit ? ONE_FREQ : ZERO_FREQ;
		amp = data->bit ? ONE_AMP : ZERO_AMP;

		out[i] = amp * sinf(data->phase / SAMPLE_RATE);
		data->phase += 2 * M_PI * frequency;
	}

	if (i < frames_per_buffer) {
		data->first = true;
		data->phase = 0.f;
                data->lstate = LSTATE_READING;
		for (; i < frames_per_buffer; i++)
			out[i] = 0.f;
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

	PaUtilRingBuffer ring_buffer;
	void *ring_buffer_ptr;

	/* Initialize ring buffer. */
	ring_buffer_ptr = malloc(RING_BUFFER_SIZE);
	if (!ring_buffer_ptr) {
		perror("malloc");
		return EXIT_FAILURE;
	}
	PaUtil_InitializeRingBuffer(&ring_buffer, 1, RING_BUFFER_SIZE,
				    ring_buffer_ptr);

	data.ring_buffer = &ring_buffer;
	data.first = true;
	data.phase = 0.f;
        data.lstate = LSTATE_READING;

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
		ring_buffer_size_t ring_ret;

		ring_ret = PaUtil_WriteRingBuffer(&ring_buffer, line, ret);
		assert(ring_ret == ret);
	}

	if (ret == -1 && !feof(stdin)) {
		perror("getline");
		status = EXIT_FAILURE;
	}

	while (PaUtil_GetRingBufferReadAvailable(&ring_buffer) > 0)
		Pa_Sleep(100); /* XXX: gross */

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
