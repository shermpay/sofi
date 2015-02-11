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

#define INTERPACKET_FRAMES (INTERPACKET_GAP * SAMPLE_RATE)

enum state {
	STATE_IDLE,
	STATE_TRANSMITTING,
	STATE_INTERPACKET_GAP,
};

struct callback_data {
	PaUtilRingBuffer *ring_buffer;
	enum state state;
	unsigned long frame;
	struct sofi_packet packet;
	size_t packet_index;
	size_t len;
	float phase;
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
	void *data1, *data2;
	ring_buffer_size_t size1, size2;

	(void)input_buffer;
	(void)time_info;
	(void)status_flags;

	for (unsigned long i = 0; i < frames_per_buffer; i++) {
		switch (data->state) {
		case STATE_IDLE:
			ring_ret = PaUtil_GetRingBufferReadRegions(data->ring_buffer,
								   MAX_PACKET_LENGTH,
								   &data1, &size1,
								   &data2, &size2);
			if (ring_ret == 0) {
				out[i] = 0.f;
				break;
			}
			memcpy(data->packet.payload, data1, size1);
			memcpy(data->packet.payload + size1, data2, size2);
			data->packet.len = size1 + size2;
			data->len = sizeof(data->packet.len) + data->packet.len;
			data->packet_index = 0;
			data->frame = SAMPLE_RATE / BAUD - 1;
			data->bit_index = 7;

			data->state = STATE_TRANSMITTING;
			/* Fallthrough. */
		case STATE_TRANSMITTING:
			if (++data->frame >= SAMPLE_RATE / BAUD) {
				if (++data->bit_index >= 8) {
					if (data->packet_index >= data->len) {
						PaUtil_AdvanceRingBufferReadIndex(data->ring_buffer,
										  data->packet.len);
						data->state = STATE_INTERPACKET_GAP;
						data->frame = 0;
						out[i] = 0.f;
						break;
					}
					data->byte = ((char *)&data->packet)[data->packet_index++];
					data->bit_index = 0;
				}
				data->bit = data->byte & (1 << data->bit_index);
				data->frame = 0;
			}

			frequency = data->bit ? ONE_FREQ : ZERO_FREQ;
			amp = data->bit ? ONE_AMP : ZERO_AMP;

			out[i] = amp * sinf(data->phase / SAMPLE_RATE);
			data->phase += 2 * M_PI * frequency;
			break;
		case STATE_INTERPACKET_GAP:
			out[i] = 0.f;
			if (++data->frame >= INTERPACKET_FRAMES)
				data->state = STATE_IDLE;
			break;
		}
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
	data.phase = 0.f;
        data.state = STATE_IDLE;

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
