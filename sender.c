#include <assert.h>
#include <math.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "portaudio.h"
#include "pa_ringbuffer.h"
#include "sofinc.h"

#define RING_BUFFER_SIZE (1 << 12)

#define SAMPLE_RATE 44100
#define FRAMES_PER_BUFFER paFramesPerBufferUnspecified

#define INTERPACKET_FRAMES (INTERPACKET_GAP * SAMPLE_RATE)

static unsigned int baud;

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
	unsigned char byte;
	unsigned int symbol_index;
	unsigned int symbol;
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
			data->frame = SAMPLE_RATE / baud - 1;
			data->symbol_index = SYMBOLS_PER_BYTE - 1;

			data->state = STATE_TRANSMITTING;
			/* Fallthrough. */
		case STATE_TRANSMITTING:
			if (++data->frame >= SAMPLE_RATE / baud) {
				if (++data->symbol_index >= SYMBOLS_PER_BYTE) {
					if (data->packet_index >= data->len) {
						PaUtil_AdvanceRingBufferReadIndex(data->ring_buffer,
										  data->packet.len);
						data->state = STATE_INTERPACKET_GAP;
						data->frame = 0;
						out[i] = 0.f;
						break;
					}
					data->byte = ((char *)&data->packet)[data->packet_index++];
					data->symbol_index = 0;
				}
				data->symbol = symbol_from_byte(data->byte, data->symbol_index);
				data->frame = 0;
			}

			frequency = symbol_freqs[data->symbol];

			out[i] = sinf(data->phase);
			data->phase += (2 * M_PI * frequency) / SAMPLE_RATE;
			while (data->phase >= 2 * M_PI)
				data->phase -= 2 * M_PI;
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

void usage() {
        printf("usage: sender -b BAUD\n");
}

int main(int argc, char **argv)
{
	PaStream *stream;
	PaError err;
	struct callback_data data;
	int status = EXIT_SUCCESS;
	char c;
	PaUtilRingBuffer ring_buffer;
	void *ring_buffer_ptr;


        if (argc != 3 || (strcmp(argv[1], "-b") != 0)) {
                usage();
                return EXIT_FAILURE;
        } else {
                char **endptr = NULL;
                int temp = strtol(argv[2], endptr, 10);
                if (endptr != NULL || temp < 1) {
                        usage();
                        printf("BAUD should be an integer between 1 to 250\n");
                        return EXIT_FAILURE;
                }
                baud = (unsigned int) temp;
        }

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

        printf("Initializing BAUD at %u\n", baud);

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


	while ((c = getc(stdin)) != EOF) {
		ring_buffer_size_t ring_ret;
		ring_ret = PaUtil_WriteRingBuffer(&ring_buffer, &c, 1);
		assert(ring_ret == 1);
	}

	if (!feof(stdin)) {
		perror("getc");
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
	free(ring_buffer_ptr);
	return status;
}
