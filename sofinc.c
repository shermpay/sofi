#include <assert.h>
#include <ctype.h>
#include <errno.h>
#include <getopt.h>
#include <inttypes.h>
#include <limits.h>
#include <math.h>
#include <pthread.h>
#include <signal.h>
#include <stdarg.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "portaudio.h"
#include "pa_ringbuffer.h"

#define M_PI 3.14159265359f

static const char *progname = "sofinc";

static int debug_mode;
void debug_printf(int v, const char *format, ...)
{
	va_list ap;

	if (debug_mode >= v) {
		va_start(ap, format);
		vfprintf(stderr, format, ap);
		va_end(ap);
	}
}

static sig_atomic_t signal_received;
static void signal_handler(int signum)
{
	signal_received = signum;
}

/* Transmission parameters. */
#define SAMPLE_RATE 44100
#define RECEIVER_WINDOW 64 /* XXX: should be in seconds, not samples. */
#define SENDER_BUFFER_SIZE (1 << 12)	/* 4K characters. */
#define RECEIVER_BUFFER_SIZE (1 << 12)	/* 4K samples. */

static float baud;
static float demod_window_factor = 0.5f;

static inline float interpacket_gap(void)
{
	return 2.f / baud;
}

static inline float delta_steady(void)
{
        return 1.f / (32.f * baud);
}

static inline float demod_window(void)
{
        return demod_window_factor / baud;
}

/* Symbol definitions. */

/* Size of a symbol in bits. XXX: (must be 1, 2, 4, or 8). */
static int symbol_width;

static inline int num_symbols(void)
{
	return 1 << symbol_width;
}

static inline unsigned int symbols_per_byte(void)
{
	return CHAR_BIT / symbol_width;
}

/* Frequencies in Hz for each symbol value. */
static float symbol_freqs[1 << 8];

static inline unsigned int symbol_from_byte(unsigned char c, unsigned int i)
{
	unsigned int mask = ((1 << symbol_width) - 1) << (symbol_width * i);
	return (c & mask) >> (symbol_width * i);
}

static inline unsigned char bits_from_symbol(unsigned int s, unsigned int i)
{
	return s << (symbol_width * i);
}

/* So-Fi state. */

#define MAX_PACKET_LENGTH 16
struct sofi_packet {
	uint8_t len;
	char payload[UINT8_MAX];
};

enum sender_state {
	SEND_STATE_IDLE,
	SEND_STATE_TRANSMITTING,
	SEND_STATE_INTERPACKET_GAP,
};

enum receiver_state {
	RECV_STATE_LISTENING,
	RECV_STATE_LENGTH_WAIT,
	RECV_STATE_LENGTH_GATHER,
	RECV_STATE_PAYLOAD_WAIT,
	RECV_STATE_PAYLOAD_GATHER,
};

struct symbol_counts {
	unsigned int symbols[1 << 8];
	unsigned int silence;
};

struct callback_data {
	struct {
		PaUtilRingBuffer buffer;
		enum sender_state state;
		unsigned long frame;
		struct sofi_packet packet;
		size_t packet_index;
		size_t len;
		float phase;
		unsigned char byte;
		unsigned int symbol_index;
		unsigned int symbol;
	} sender;

	struct {
		PaUtilRingBuffer buffer;
	} receiver;
};

static int sofi_callback(const void *input_buffer, void *output_buffer,
			 unsigned long frames_per_buffer,
			 const PaStreamCallbackTimeInfo *time_info,
			 PaStreamCallbackFlags status_flags, void *arg)
{
	ring_buffer_size_t ret;
	float *out = output_buffer;
	struct callback_data *data = arg;
	float frequency;
	void *data1, *data2;
	ring_buffer_size_t size1, size2;
	bool first = false;

	(void)time_info;
	(void)status_flags;

	for (unsigned long i = 0; i < frames_per_buffer; i++) {
		switch (data->sender.state) {
		case SEND_STATE_IDLE:
			ret = PaUtil_GetRingBufferReadRegions(&data->sender.buffer,
							      MAX_PACKET_LENGTH,
							      &data1, &size1,
							      &data2, &size2);
			if (ret == 0) {
				out[i] = 0.f;
				break;
			}
			memcpy(data->sender.packet.payload, data1, size1);
			memcpy(data->sender.packet.payload + size1, data2, size2);
			data->sender.packet.len = size1 + size2;
			data->sender.len = sizeof(data->sender.packet.len) + data->sender.packet.len;
			data->sender.packet_index = 0;

			first = true;
			data->sender.state = SEND_STATE_TRANSMITTING;
			/* Fallthrough. */
		case SEND_STATE_TRANSMITTING:
			if (first || ++data->sender.frame >= SAMPLE_RATE / baud) {
				if (first || ++data->sender.symbol_index >= symbols_per_byte()) {
					if (data->sender.packet_index >= data->sender.len) {
						PaUtil_AdvanceRingBufferReadIndex(&data->sender.buffer,
										  data->sender.packet.len);
						data->sender.state = SEND_STATE_INTERPACKET_GAP;
						data->sender.frame = 0;
						out[i] = 0.f;
						break;
					}
					data->sender.byte = ((char *)&data->sender.packet)[data->sender.packet_index++];
					data->sender.symbol_index = 0;
				}
				data->sender.symbol = symbol_from_byte(data->sender.byte, data->sender.symbol_index);
				data->sender.frame = 0;
			}

			frequency = symbol_freqs[data->sender.symbol];

			out[i] = sinf(data->sender.phase);
			data->sender.phase += (2 * M_PI * frequency) / SAMPLE_RATE;
			while (data->sender.phase >= 2 * M_PI)
				data->sender.phase -= 2 * M_PI;
			first = false;
			break;
		case SEND_STATE_INTERPACKET_GAP:
			out[i] = 0.f;
			if (++data->sender.frame >= interpacket_gap() * SAMPLE_RATE)
				data->sender.state = SEND_STATE_IDLE;
			break;
		}
	}

	if (data->sender.state == SEND_STATE_IDLE) {
		ret = PaUtil_GetRingBufferWriteAvailable(&data->receiver.buffer);
		assert((unsigned long)ret >= frames_per_buffer);
		ret = PaUtil_WriteRingBuffer(&data->receiver.buffer, input_buffer, frames_per_buffer);
		assert((unsigned long)ret == frames_per_buffer);
	}

	return paContinue;
}

static void sender_loop(PaUtilRingBuffer *buffer)
{
	ring_buffer_size_t ring_ret;
	char c;

	for (;;) {
		c = getc(stdin);
		if (c == EOF) {
			if ((errno = ferror(stdin))) {
				perror("getc");
				break;
			}
			continue;
		}
		ring_ret = PaUtil_WriteRingBuffer(buffer, &c, 1);
		assert(ring_ret == 1);
	}
}

static void *sender_start(void *arg)
{
	sender_loop(arg);
	return NULL;
}

static inline int strongest_symbol(const float *fs)
{
	int max_symbol = -1;
	float max_val = 0.f;

	for (int i = 0; i < num_symbols(); i++) {
		/* XXX: need a real heuristic for silence. */
		if (fs[i] > 100.f && fs[i] > max_val) {
			max_val = fs[i];
			max_symbol = i;
		}
	}
	return max_symbol;
}

/* XXX: convert window to frame to time in seconds. */
static inline float window_to_seconds(int window)
{
	return (float)window * (float)RECEIVER_WINDOW / (float)SAMPLE_RATE;
}

static inline int calc_mostly(struct symbol_counts *counts)
{
	int max = -1;
	unsigned int max_count = counts->silence;

	for (int i = 0; i < num_symbols(); i++) {
		if (counts->symbols[i] > max_count) {
			max_count = counts->symbols[i];
			max = i;
		}
	}
	return max;
}

static void print_frame(const struct sofi_packet *packet)
{
	if (debug_mode < 1) {
		fwrite(packet->payload, 1, packet->len, stdout);
		return;
	}

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

static void receiver_loop(PaUtilRingBuffer *buffer, float *window_buffer)
{
	ring_buffer_size_t ring_ret;
	int signum;
	int window = 0;
	enum receiver_state state = RECV_STATE_LISTENING;
	float t0;
	float wait_until;
	int prev = -1;
	int n;
	struct symbol_counts counts;
	char byte = 0;
	unsigned int symbol_index = 0;
	struct sofi_packet packet;
	unsigned offset;
	float sinfs[1 << 8];
	float cosfs[1 << 8];
	float fs[1 << 8];
	float time;
	int symbol;

#define RECV_STATE_TRANSITION(new_state) do {	\
	state = new_state;			\
	debug_printf(2, "%s\n", #new_state);	\
} while(0)

	while (!(signum = signal_received)) {
		if (PaUtil_GetRingBufferReadAvailable(buffer) < RECEIVER_WINDOW)
			continue;

		time = window_to_seconds(window++);

		ring_ret = PaUtil_ReadRingBuffer(buffer, window_buffer,
						 RECEIVER_WINDOW);
		assert(ring_ret == RECEIVER_WINDOW);

		memset(sinfs, 0, sizeof(sinfs));
		memset(cosfs, 0, sizeof(cosfs));
		for (int i = 0; i < RECEIVER_WINDOW; i++) {
			for (int j = 0; j < num_symbols(); j++) {
				sinfs[j] += sinf(2 * M_PI * symbol_freqs[j] * i / SAMPLE_RATE) * window_buffer[i];
				cosfs[j] += cosf(2 * M_PI * symbol_freqs[j] * i / SAMPLE_RATE) * window_buffer[i];
			}
		}
		for (int j = 0; j < num_symbols(); j++)
			fs[j] = sinfs[j] * sinfs[j] + cosfs[j] * cosfs[j];
		symbol = strongest_symbol(fs);

		switch (state) {
		case RECV_STATE_LISTENING:
			if (symbol != prev) {
				t0 = time;
				packet.len = 0;
				offset = 0;
				n = 0;
				byte = 0;
				symbol_index = 0;
				wait_until = t0 + (1.f / (2.f * baud)) + (n / (float)baud) - (demod_window() / 2.f);
				RECV_STATE_TRANSITION(RECV_STATE_LENGTH_WAIT);
			}
			break;
		case RECV_STATE_LENGTH_WAIT:
		case RECV_STATE_PAYLOAD_WAIT:
			if (time >= wait_until) {
				memset(&counts, 0, sizeof(counts));
				wait_until = t0 + (1.f / (2.f * baud)) + (n / baud) + (demod_window() / 2.f);
				if (state == RECV_STATE_LENGTH_WAIT)
					RECV_STATE_TRANSITION(RECV_STATE_LENGTH_GATHER);
				else if (state == RECV_STATE_PAYLOAD_WAIT)
					RECV_STATE_TRANSITION(RECV_STATE_PAYLOAD_GATHER);
			}
			break;
		case RECV_STATE_LENGTH_GATHER:
		case RECV_STATE_PAYLOAD_GATHER:
			if (time >= wait_until) {
				int mostly;

				n++;

				mostly = calc_mostly(&counts);
				if (mostly == -1) {
					memset(packet.payload + offset, 0, packet.len - offset);
					print_frame(&packet);
					wait_until = t0 + (n / baud) + interpacket_gap();
					RECV_STATE_TRANSITION(RECV_STATE_LISTENING);
					break;
				}

				wait_until = t0 + (1.f / (2.f * baud)) + (n / baud) - (demod_window() / 2.f);

				byte |= bits_from_symbol(mostly, symbol_index++);
				if (symbol_index >= symbols_per_byte()) {
					if (state == RECV_STATE_LENGTH_GATHER) {
						packet.len = (uint8_t)byte;
					} else if (state == RECV_STATE_PAYLOAD_GATHER) {
						if (offset < packet.len &&
						    offset < MAX_PACKET_LENGTH)
							packet.payload[offset++] = byte;
					}
					RECV_STATE_TRANSITION(RECV_STATE_PAYLOAD_WAIT);
					byte = 0;
					symbol_index = 0;
				} else {
					if (state == RECV_STATE_LENGTH_GATHER)
						RECV_STATE_TRANSITION(RECV_STATE_LENGTH_WAIT);
					else if (state == RECV_STATE_PAYLOAD_GATHER)
						RECV_STATE_TRANSITION(RECV_STATE_PAYLOAD_WAIT);
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

static void usage(bool error)
{
	fprintf(error ? stderr : stdout,
		"Usage: %1$s [-d] [-f FREQ1,FREQ2,...] [-w DEMOD_WINDOW] -b BAUD\n"
		"       %1$s -h\n", progname);
	exit(error ? EXIT_FAILURE : EXIT_SUCCESS);
}

int main(int argc, char **argv)
{
	PaStream *stream;
	PaError err;
	int status = EXIT_SUCCESS;
	struct callback_data data;
	void *sender_buffer_ptr = NULL;
	void *receiver_buffer_ptr = NULL;
	float *receiver_window = NULL;
	pthread_t sender_thread;
	int ret;
	int opt;

	if (argc > 0)
		progname = argv[0];
	while ((opt = getopt(argc, argv, "db:f:w:h")) != -1) {
		char *end;
		long temp;
		float freq;
		int i;

		switch (opt) {
		case 'b':
			temp = strtol(optarg, &end, 10);
			if (*end != '\0')
				usage(true);
			baud = (float)temp;
			break;
                case 'w':
                        demod_window_factor = strtof(optarg, &end);
			if (*end != '\0')
				usage(true);
			break;
		case 'd':
			debug_mode++;
			break;
		case 'f':
			for (i = 0; i < 256; i++) {
				if (*optarg == '\0')
					usage(true);
				freq = strtof(optarg, &end);
				if (*end != '\0' && *end != ',')
					usage(true);
				symbol_freqs[i] = freq;
				if (*end == '\0')
					break;
				else
					optarg = end + 1;
			}
			if (i == 1) {
				symbol_width = 1;
			} else if (i == 3) {
				symbol_width = 2;
			} else if (i == 15) {
				symbol_width = 4;
			} else if (i == 255) {
				symbol_width = 8;
			} else {
				fprintf(stderr, "%s: symbol width must be 1, 2, 4, or 8\n",
					progname);
				usage(true);
			}
			break;
		case 'h':
			usage(false);
		default:
			usage(true);
		}
	}
	if (baud < 1.f)
		usage(true);

	if (symbol_width == 0) {
		symbol_width = 1;
		symbol_freqs[0] = 2200.f;
		symbol_freqs[1] = 1200.f;
	}

	/* Initialize callback data and receiver window buffer. */
	sender_buffer_ptr = malloc(SENDER_BUFFER_SIZE);
	receiver_buffer_ptr = malloc(RECEIVER_BUFFER_SIZE * sizeof(float));
	if (!sender_buffer_ptr || !receiver_buffer_ptr) {
		perror("malloc");
		status = EXIT_FAILURE;
		goto out;
	}
	PaUtil_InitializeRingBuffer(&data.sender.buffer, 1, SENDER_BUFFER_SIZE,
				    sender_buffer_ptr);
	PaUtil_InitializeRingBuffer(&data.receiver.buffer, sizeof(float),
				    RECEIVER_BUFFER_SIZE, receiver_buffer_ptr);
	data.sender.phase = 0.f;
        data.sender.state = SEND_STATE_IDLE;

	receiver_window = calloc(RECEIVER_WINDOW, sizeof(float));
	if (!receiver_window) {
		perror("calloc");
		status = EXIT_FAILURE;
		goto out;
	}

	/* Handle signals. */
	if (signal(SIGINT, signal_handler) == SIG_ERR) {
		perror("signal");
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

	/* Open a duplex stream and start it. */
	err = Pa_OpenDefaultStream(&stream, 1, 1, paFloat32, SAMPLE_RATE,
				   paFramesPerBufferUnspecified, sofi_callback,
				   &data);
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
		goto stop_stream;
	}

	ret = pthread_create(&sender_thread, NULL, sender_start,
			     &data.sender.buffer);
	if (ret) {
		errno = ret;
		perror("pthread_create");
		status = EXIT_FAILURE;
		goto close_stream;
	}
	receiver_loop(&data.receiver.buffer, receiver_window);
	pthread_cancel(sender_thread);

	/* Cleanup. */
stop_stream:
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
	free(sender_buffer_ptr);
	free(receiver_buffer_ptr);
	free(receiver_window);
	return status;
}
