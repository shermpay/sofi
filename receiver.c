#include <assert.h>
#include <complex.h>
#include <ctype.h>
#include <math.h>
#include <signal.h>
#include <stdarg.h>
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

#define FFT_WINDOW 1024
#define ACTUAL_WINDOW 256
#define SLIDE_WINDOW (ACTUAL_WINDOW / 4)

#define DELTA_STEADY (1.f / (32.f * baud))
#define DEMOD_WINDOW (1.f / (2.f * baud))

static bool debug_mode;
static float baud;

static inline float delta_steady(int baud) {
        return (1.f / (32.f * baud));
}

static inline float demod_window(int baud) {
        return (1.f / (2.f * baud));
}

void debug_printf(const char *format, ...)
{
	va_list ap;

	if (debug_mode) {
		va_start(ap, format);
		vfprintf(stderr, format, ap);
		va_end(ap);
	}
}

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

enum state {
	STATE_LISTENING,
	STATE_LENGTH_WAIT,
	STATE_LENGTH_GATHER,
	STATE_PAYLOAD_WAIT,
	STATE_PAYLOAD_GATHER,
};

static inline int strongest_symbol(const float *fs)
{
	int max_symbol = -1;
	float max_val = 0.f;

	for (int i = 0; i < NUM_SYMBOLS; i++) {
		/* XXX: need a real heuristic for silence. */
		if (fs[i] > 100.f && fs[i] > max_val) {
			max_val = fs[i];
			max_symbol = i;
		}
	}
	return max_symbol;
}

struct symbol_counts {
	unsigned int symbols[NUM_SYMBOLS];
	unsigned int silence;
};

static inline int calc_mostly(struct symbol_counts *counts)
{
	int max = -1;
	unsigned int max_count = counts->silence;

	for (int i = 0; i < NUM_SYMBOLS; i++) {
		if (counts->symbols[i] > max_count) {
			max_count = counts->symbols[i];
			max = i;
		}
	}
	return max;
}

static void print_frame(const struct sofi_packet *packet)
{
	if (packet->len == 0)
		return;
	printf("sofi_frame = {\n");
	printf("\t.len = %" PRIu8 "\n", packet->len);
	printf("\t.payload = \"");
	for (unsigned i = 0; i < packet->len; i++) {
		char c = packet->payload[i];
		switch (c) {
		case '\"':
			fputs("\\\"", stdout);
			break;
		case '\\':
			fputs("\\\\", stdout);
			break;
		case '\a':
			fputs("\\a", stdout);
			break;
		case '\b':
			fputs("\\b", stdout);
			break;
		case '\n':
			fputs("\\n", stdout);
			break;
		case '\t':
			fputs("\\t", stdout);
			break;
		default:
			if (isprint(c))
				fputc(c, stdout);
			else
				printf("\\%03o", (unsigned char)c);
		}
	}
	printf("\"\n");
	printf("}\n");
	fflush(stdout);
}

static void receiver_loop(PaUtilRingBuffer *ring_buffer, float *sample_buf)
{
	ring_buffer_size_t ring_ret;
	int signum;
	int window = 0;
	enum state state = STATE_LISTENING;
	float t0;
	float wait_until;
	int prev = -1;
	int n;
	struct symbol_counts counts;
	char byte = 0;
	int symbol_index = 0;
	struct sofi_packet packet;
	unsigned offset;

#define STATE_TRANSITION(new_state) do {	\
	state = new_state;			\
	debug_printf("%s\n", #new_state);	\
} while(0)

	while (!(signum = signal_received)) {
		float sinfs[NUM_SYMBOLS];
		float cosfs[NUM_SYMBOLS];
		float fs[NUM_SYMBOLS];
		float time;
		int symbol;

		if (PaUtil_GetRingBufferReadAvailable(ring_buffer) < SLIDE_WINDOW)
			continue;

		/* Compute the current value in the raw stream. */
		time = window_to_seconds(window++);

		ring_ret = PaUtil_ReadRingBuffer(ring_buffer,
						 sample_buf,
						 SLIDE_WINDOW);
		assert(ring_ret == SLIDE_WINDOW);

		memset(sinfs, 0, sizeof(sinfs));
		memset(cosfs, 0, sizeof(cosfs));
		for (int i = 0; i < SLIDE_WINDOW; i++) {
			for (int j = 0; j < NUM_SYMBOLS; j++) {
				sinfs[j] += sinf(2 * M_PI * symbol_freqs[j] * i / SAMPLE_RATE) * sample_buf[i];
				cosfs[j] += cosf(2 * M_PI * symbol_freqs[j] * i / SAMPLE_RATE) * sample_buf[i];
			}
		}
		for (int j = 0; j < NUM_SYMBOLS; j++)
			fs[j] = sinfs[j] * sinfs[j] + cosfs[j] * cosfs[j];
		symbol = strongest_symbol(fs);

		switch (state) {
		case STATE_LISTENING:
			if (symbol != prev) {
				t0 = time;
				packet.len = 0;
				offset = 0;
				n = 0;
				byte = 0;
				symbol_index = 0;
				wait_until = t0 + (1.f / (2.f * baud)) + (n / (float)baud) - (demod_window(baud) / 2.f);
				STATE_TRANSITION(STATE_LENGTH_WAIT);
			}
			break;
		case STATE_LENGTH_WAIT:
		case STATE_PAYLOAD_WAIT:
			if (time >= wait_until) {
				memset(&counts, 0, sizeof(counts));
				wait_until = t0 + (1.f / (2.f * baud)) + (n / (float)baud) + (demod_window(baud) / 2.f);
				if (state == STATE_LENGTH_WAIT)
					STATE_TRANSITION(STATE_LENGTH_GATHER);
				else if (state == STATE_PAYLOAD_WAIT)
					STATE_TRANSITION(STATE_PAYLOAD_GATHER);
			}
			break;
		case STATE_LENGTH_GATHER:
		case STATE_PAYLOAD_GATHER:
			if (time >= wait_until) {
				int mostly;

				n++;

				mostly = calc_mostly(&counts);
				if (mostly == -1) {
					memset(packet.payload + offset, 0, packet.len - offset);
					print_frame(&packet);
					wait_until = t0 + (n / (float)baud) + INTERPACKET_GAP;
					STATE_TRANSITION(STATE_LISTENING);
					break;
				}

				wait_until = t0 + (1.f / (2.f * baud)) + (n / (float)baud) - (demod_window(baud) / 2.f);

				debug_printf("bit = %d\n", mostly);
				byte |= bits_from_symbol(mostly, symbol_index++);
				if (symbol_index >= SYMBOLS_PER_BYTE) {
					if (state == STATE_LENGTH_GATHER) {
						packet.len = (uint8_t)byte;
					} else if (state == STATE_PAYLOAD_GATHER) {
						if (offset < packet.len &&
						    offset < MAX_PACKET_LENGTH)
							packet.payload[offset++] = byte;
					}
					STATE_TRANSITION(STATE_PAYLOAD_WAIT);
					byte = 0;
					symbol_index = 0;
				} else {
					if (state == STATE_LENGTH_GATHER)
						STATE_TRANSITION(STATE_LENGTH_WAIT);
					else if (state == STATE_PAYLOAD_GATHER)
						STATE_TRANSITION(STATE_PAYLOAD_WAIT);
				}
			} else {
				if (symbol == -1)
					counts.silence++;
				else
					counts.symbols[symbol]++;
			}
			break;
		}

		prev = symbol;
	}

	fprintf(stderr, "got %d; exiting\n", signum);
}

void usage() {
        printf("usage: receiver -b BAUD\n");
}

int main(int argc, char **argv)
{
	float *sample_buf = NULL;

	PaUtilRingBuffer ring_buffer;
	void *ring_buffer_ptr;

	PaStream *stream;
	PaError err;

	int status = EXIT_SUCCESS;

        if (argc != 3 || (strcmp(argv[1], "-b") != 0)) {
                usage();
                return EXIT_FAILURE;
        } else {
                char **endptr = NULL;
                int temp = strtol(argv[2], endptr, 10);
                if (endptr != NULL || temp < 1) {
                        usage();
                        printf("baud should be an integer between 1 to 250\n");
                        return EXIT_FAILURE;
                }
                baud = (float)temp;
        }

	/* Handle signals. */
	signal(SIGINT, signal_handler);

	/* Initialize ring buffer. */
	ring_buffer_ptr = malloc(RING_BUFFER_SIZE * sizeof(float));
	if (!ring_buffer_ptr) {
		perror("malloc");
		return EXIT_FAILURE;
	}
	PaUtil_InitializeRingBuffer(&ring_buffer, sizeof(float), RING_BUFFER_SIZE,
				    ring_buffer_ptr);

	/* Allocate buffer for data. */
	sample_buf = calloc(ACTUAL_WINDOW, sizeof(float));
	if (!sample_buf) {
		perror("calloc");
		status = EXIT_FAILURE;
		goto out;
	}

	/* Initialize PortAudio. */
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

	/* Run the receiver. */
	err = Pa_StartStream(stream);
	if (err != paNoError) {
		fprintf(stderr, "PortAudio: starting stream failed: %s\n",
			Pa_GetErrorText(err));
		status = EXIT_FAILURE;
		goto close_stream;
	}

	receiver_loop(&ring_buffer, sample_buf);

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

out:
	free(sample_buf);
	free(ring_buffer_ptr);
	return status;
}
