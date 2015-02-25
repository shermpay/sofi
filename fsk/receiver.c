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
#include <unistd.h>

#include "portaudio.h"
#include "pa_ringbuffer.h"
#include "fsk.h"

#define RING_BUFFER_SIZE (1 << 12)

#define SAMPLE_RATE 44100
#define FRAMES_PER_BUFFER paFramesPerBufferUnspecified

#define FFT_WINDOW 1024
#define ACTUAL_WINDOW 256
#define SLIDE_WINDOW (ACTUAL_WINDOW / 4)

#define DELTA_STEADY (1.f / (32.f * BAUD))
#define DEMOD_WINDOW (1.f / (2.f * BAUD))

static bool debug_mode;

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

struct sig_counts {
	int one;
	int zero;
	int none;
};

static inline int calc_mostly(struct sig_counts *counts)
{
	if (counts->zero > counts->one && counts->zero > counts->none)
		return 0;
	else if (counts->one > counts->zero && counts->one > counts->none)
		return 1;
	else
		return -1;
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
	struct sig_counts counts;
	char byte = 0;
	int bit_index = 0;
	struct sofi_packet packet;
	unsigned offset;

#define STATE_TRANSITION(new_state) do {	\
	state = new_state;			\
	debug_printf("%s\n", #new_state);	\
} while(0)

	while (!(signum = signal_received)) {
		float time;
		int val;

		if (PaUtil_GetRingBufferReadAvailable(ring_buffer) < SLIDE_WINDOW)
			continue;

		/* Compute the current value in the raw stream. */
		time = window_to_seconds(window++);

		ring_ret = PaUtil_ReadRingBuffer(ring_buffer,
						 sample_buf,
						 SLIDE_WINDOW);
		assert(ring_ret == SLIDE_WINDOW);

		{
			float sinf0 = 0.f, cosf0 = 0.f;
			float sinf1 = 0.f, cosf1 = 0.f;
			float f0, f1;

			for (int i = 0; i < SLIDE_WINDOW; i++) {
				sinf0 += sin(2 * M_PI * ZERO_FREQ * i / SAMPLE_RATE) * sample_buf[i];
				cosf0 += cos(2 * M_PI * ZERO_FREQ * i / SAMPLE_RATE) * sample_buf[i];
				sinf1 += sin(2 * M_PI * ONE_FREQ * i / SAMPLE_RATE) * sample_buf[i];
				cosf1 += cos(2 * M_PI * ONE_FREQ * i / SAMPLE_RATE) * sample_buf[i];
			}

			f0 = sinf0 * sinf0 + cosf0 * cosf0;
			f1 = sinf1 * sinf1 + cosf1 * cosf1;

			fprintf(stderr, "0\t%f\t%f\n", time, f0);
			fprintf(stderr, "1\t%f\t%f\n", time, f1);

			if (f0 < 100.f && f1 < 100.f)
				val = 0;
			else if (f0 > f1)
				val = -1;
			else
				val = 1;
			fprintf(stderr, "2\t%f\t%d\n", time, val);
		}

		switch (state) {
		case STATE_LISTENING:
			if (val != prev) {
				t0 = time;
				packet.len = 0;
				offset = 0;
				n = 0;
				byte = 0;
				bit_index = 0;
				wait_until = t0 + (1.f / (2.f * BAUD)) + (n / (float)BAUD) - (DEMOD_WINDOW / 2.f);
				STATE_TRANSITION(STATE_LENGTH_WAIT);
			}
			break;
		case STATE_LENGTH_WAIT:
		case STATE_PAYLOAD_WAIT:
			if (time >= wait_until) {
				counts.zero = 0;
				counts.one = 0;
				counts.none = 0;
				wait_until = t0 + (1.f / (2.f * BAUD)) + (n / (float)BAUD) + (DEMOD_WINDOW / 2.f);
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
				if (mostly != 0 && mostly != 1) {
					memset(packet.payload + offset, 0, packet.len - offset);
					print_frame(&packet);
					wait_until = t0 + (n / (float)BAUD) + INTERPACKET_GAP;
					STATE_TRANSITION(STATE_LISTENING);
					break;
				}

				wait_until = t0 + (1.f / (2.f * BAUD)) + (n / (float)BAUD) - (DEMOD_WINDOW / 2.f);

				debug_printf("bit = %d\n", mostly);
				byte |= mostly << bit_index++;
				if (bit_index == 8) {
					if (state == STATE_LENGTH_GATHER) {
						packet.len = (uint8_t)byte;
					} else if (state == STATE_PAYLOAD_GATHER) {
						if (offset < packet.len &&
						    offset < MAX_PACKET_LENGTH)
							packet.payload[offset++] = byte;
					}
					STATE_TRANSITION(STATE_PAYLOAD_WAIT);
					byte = 0;
					bit_index = 0;
				} else {
					if (state == STATE_LENGTH_GATHER)
						STATE_TRANSITION(STATE_LENGTH_WAIT);
					else if (state == STATE_PAYLOAD_GATHER)
						STATE_TRANSITION(STATE_PAYLOAD_WAIT);
				}
			} else {
				if (val == -1)
					counts.zero++;
				else if (val == 1)
					counts.one++;
				else
					counts.none++;
			}
			break;
		}

		prev = val;
	}

	fprintf(stderr, "got %s; exiting\n", strsignal(signum));
}

int main(int argc, char **argv)
{
	float *sample_buf = NULL;

	PaUtilRingBuffer ring_buffer;
	void *ring_buffer_ptr;

	PaStream *stream;
	PaError err;

	int status = EXIT_SUCCESS;
	struct sigaction sa;

	int opt;

	while ((opt = getopt(argc, argv, "d")) != -1) {
		switch (opt) {
		case 'd':
			debug_mode = true;
			break;
		default:
			return EXIT_FAILURE;
		}
	}

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
