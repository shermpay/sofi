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
#define SENDER_BUFFER_SIZE (1 << 12)	/* 4K characters. */
#define RECEIVER_BUFFER_SIZE (1 << 20)	/* 1M samples. */

static long sample_rate = 44100;
static float baud;
static float recv_window_factor = 0.2f;

static inline int receiver_window(void)
{
	return (int)(recv_window_factor / baud * (float)sample_rate);
}

static inline float interpacket_gap(void)
{
	return 2.f / baud;
}

/* Symbol definitions. */

/* Size of a symbol in bits. XXX: (must be 1, 2, 4, or 8). */
static int symbol_width = 1;

static inline int num_symbols(void)
{
	return 1 << symbol_width;
}

static inline unsigned int symbols_per_byte(void)
{
	return CHAR_BIT / symbol_width;
}

/* Frequencies in Hz for each symbol value. */
static float symbol_freqs[1 << 8] = {2200.f, 1200.f};

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
	RECV_STATE_LENGTH_GATHER,
	RECV_STATE_PAYLOAD_GATHER,
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

static void sender_callback(void *output_buffer,
			    unsigned long frames_per_buffer,
			    struct callback_data *data)
{
	ring_buffer_size_t ret;
	float *out = output_buffer;
	float frequency;
	void *data1, *data2;
	ring_buffer_size_t size1, size2;
	bool first = false;

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
			if (first || ++data->sender.frame >= sample_rate / baud) {
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
			data->sender.phase += (2 * M_PI * frequency) / sample_rate;
			while (data->sender.phase >= 2 * M_PI)
				data->sender.phase -= 2 * M_PI;
			first = false;
			break;
		case SEND_STATE_INTERPACKET_GAP:
			out[i] = 0.f;
			if (++data->sender.frame >= interpacket_gap() * sample_rate)
				data->sender.state = SEND_STATE_IDLE;
			break;
		}
	}
}

static void receiver_callback(const void *input_buffer,
			      unsigned long frames_per_buffer,
			      struct callback_data *data)
{
	ring_buffer_size_t ret;
	if (data->sender.state == SEND_STATE_IDLE) {
		ret = PaUtil_GetRingBufferWriteAvailable(&data->receiver.buffer);
		assert((unsigned long)ret >= frames_per_buffer);
		ret = PaUtil_WriteRingBuffer(&data->receiver.buffer, input_buffer, frames_per_buffer);
		assert((unsigned long)ret == frames_per_buffer);
	}
}

static int sofi_callback(const void *input_buffer, void *output_buffer,
			 unsigned long frames_per_buffer,
			 const PaStreamCallbackTimeInfo *time_info,
			 PaStreamCallbackFlags status_flags, void *arg)
{

	(void)time_info;
	(void)status_flags;

	if (output_buffer)
		sender_callback(output_buffer, frames_per_buffer, arg);
	if (input_buffer)
		receiver_callback(input_buffer, frames_per_buffer, arg);

	return paContinue;
}

static int sender_loop(PaUtilRingBuffer *buffer)
{
	ring_buffer_size_t ring_ret;
	char c;
	int ret = 0;

	for (;;) {
		c = getc(stdin);
		if (c == EOF) {
			if (ferror(stdin) && errno != EINTR) {
				perror("getc");
				ret = -1;
			}
			break;
		}
		while (PaUtil_GetRingBufferWriteAvailable(buffer) < 1)
			Pa_Sleep(CHAR_BIT * 1000.f / baud);
		ring_ret = PaUtil_WriteRingBuffer(buffer, &c, 1);
		assert(ring_ret == 1);
	}

	/* Wait for any outstanding output to be sent. */
	while (PaUtil_GetRingBufferReadAvailable(buffer) > 0)
		Pa_Sleep(100);
	return ret;
}

static void *sender_start(void *arg)
{
	return (void *)(uintptr_t)sender_loop(arg);
}

static int print_frame(const struct sofi_packet *packet)
{
	if (debug_mode < 1) {
		fwrite(packet->payload, 1, packet->len, stdout);
	} else {
		if (packet->len == 0 && debug_mode < 2)
			return 0;
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
	}
	fflush(stdout);
	if (ferror(stdout)) {
		perror("fflush");
		return -1;
	}
	return 0;
}

static int receiver_loop(PaUtilRingBuffer *buffer, float *window_buffer)
{
	ring_buffer_size_t ring_ret;
	int signum;
	enum receiver_state state = RECV_STATE_LISTENING;
	char byte = 0;
	unsigned int symbol_index = 0;
	struct sofi_packet packet;
	unsigned offset = 0;
	int symbol;
	float max_strength;

#define RECV_STATE_TRANSITION(new_state) do {	\
	state = new_state;			\
	debug_printf(2, "%s\n", #new_state);	\
} while(0)

	while (!(signum = signal_received)) {
		int window_size;

		if (state == RECV_STATE_LISTENING)
			window_size = receiver_window();
		else
			window_size = (int)((float)sample_rate / baud);

		if (PaUtil_GetRingBufferReadAvailable(buffer) < window_size) {
			Pa_Sleep(1000 * window_size / sample_rate);
			continue;
		}

		ring_ret = PaUtil_ReadRingBuffer(buffer, window_buffer,
						 window_size);
		assert(ring_ret == window_size);

		debug_printf(3, "symbol strengths = [");
		symbol = -1;
		max_strength = 100.f; /* XXX: need a real heuristic for silence. */
		for (int i = 0; i < num_symbols(); i++) {
			float sin_i = 0.f, cos_i = 0.f;
			float strength;

			for (int j = 0; j < window_size; j++) {
				sin_i += sinf(2.f * M_PI * symbol_freqs[i] * (float)j / (float)sample_rate) * window_buffer[j];
				cos_i += cosf(2.f * M_PI * symbol_freqs[i] * (float)j / (float)sample_rate) * window_buffer[j];
			}
			strength = sin_i * sin_i + cos_i * cos_i;
			if (strength > max_strength) {
				max_strength = strength;
				symbol = i;
			}

			if (i > 0)
				debug_printf(3, ", ");
			debug_printf(3, "%f", strength);
		}
		debug_printf(3, "] = %d\n", symbol);

		switch (state) {
		case RECV_STATE_LISTENING:
			if (symbol != -1) {
				packet.len = 0;
				offset = 0;
				byte = 0;
				symbol_index = 0;
				RECV_STATE_TRANSITION(RECV_STATE_LENGTH_GATHER);
			}
			break;
		case RECV_STATE_LENGTH_GATHER:
		case RECV_STATE_PAYLOAD_GATHER:
			if (symbol == -1) {
				memset(packet.payload + offset, 0, packet.len - offset);
				if (print_frame(&packet))
					return -1;
				RECV_STATE_TRANSITION(RECV_STATE_LISTENING);
				break;
			}

			byte |= bits_from_symbol(symbol, symbol_index++);
			if (symbol_index >= symbols_per_byte()) {
				if (state == RECV_STATE_LENGTH_GATHER) {
					packet.len = (uint8_t)byte;
					RECV_STATE_TRANSITION(RECV_STATE_PAYLOAD_GATHER);
				} else if (state == RECV_STATE_PAYLOAD_GATHER) {
					if (offset < packet.len &&
					    offset < MAX_PACKET_LENGTH)
						packet.payload[offset++] = byte;
				}
				byte = 0;
				symbol_index = 0;
			}
			break;
		}
	}

	fprintf(stderr, "got %d; exiting\n", signum);
	return 0;
}

static void usage(bool error)
{
	fprintf(error ? stderr : stdout,
		"Usage: %s [-d] [-RS] [-f FREQ1,FREQ2,...] [-s SAMPLE RATE] [-w RECV WINDOW] -b BAUD\n"
		"       %s -h\n", progname, progname);
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
	float *window_buffer = NULL;
	pthread_t sender_thread;
	int ret;
	int opt;
	PaStreamParameters input_params, output_params;
	bool sender = false, receiver = false;

	if (argc > 0)
		progname = argv[0];
	while ((opt = getopt(argc, argv, "RSb:df:s:w:h")) != -1) {
		char *end;
		float freq;
		int i;

		switch (opt) {
		case 'R':
			receiver = true;
			break;
		case 'S':
			sender = true;
			break;
		case 'b':
			baud = strtof(optarg, &end);
			if (*end != '\0')
				usage(true);
			if (baud < 1.f) {
				fprintf(stderr, "%s: baud must be >=1\n",
					progname);
				usage(true);
			}
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
		case 's':
			sample_rate = strtol(optarg, &end, 10);
			if (*end != '\0')
				usage(true);
			if (sample_rate <= 0) {
				fprintf(stderr, "%s: sample rate must be positive\n",
					progname);
				usage(true);
			}
			break;
		case 'w':
			recv_window_factor = strtof(optarg, &end);
			if (*end != '\0')
				usage(true);
			if (recv_window_factor <= 0.f) {
				fprintf(stderr, "%s: receiver window factor must be positive\n",
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
	if (!baud)
		usage(true);
	if (!sender && !receiver)
		sender = receiver = true;
	if (debug_mode > 0) {
		fprintf(stderr,
			"Sample rate:\t%ld Hz\n"
			"Baud:\t\t%.2f symbols/sec, %d samples, %.2f seconds\n"
			"Window:\t\t%d samples, %.2f seconds\n",
			sample_rate,
			baud, (int)((float)sample_rate / baud), 1.f / baud,
			receiver_window(), receiver_window() / (float)sample_rate);
		fprintf(stderr, "Frequencies:\t");
		for (int i = 0; i < num_symbols(); i++) {
			if (i > 0)
				fprintf(stderr, ", ");
			fprintf(stderr, "%.2f Hz", symbol_freqs[i]);
		}
		fprintf(stderr, "\n");
	}

	/* Initialize callback data and receiver window buffer. */
	if (sender) {
		sender_buffer_ptr = malloc(SENDER_BUFFER_SIZE);
		if (!sender_buffer_ptr) {
			perror("malloc");
			status = EXIT_FAILURE;
			goto out;
		}
		PaUtil_InitializeRingBuffer(&data.sender.buffer, 1,
					    SENDER_BUFFER_SIZE,
					    sender_buffer_ptr);
		data.sender.phase = 0.f;
	}
	if (receiver) {
		receiver_buffer_ptr = malloc(RECEIVER_BUFFER_SIZE * sizeof(float));
		if (!receiver_buffer_ptr) {
			perror("malloc");
			status = EXIT_FAILURE;
			goto out;
		}
		PaUtil_InitializeRingBuffer(&data.receiver.buffer,
					    sizeof(float), RECEIVER_BUFFER_SIZE,
					    receiver_buffer_ptr);
		window_buffer = malloc(RECEIVER_BUFFER_SIZE * sizeof(float));
		if (!window_buffer) {
			perror("malloc");
			status = EXIT_FAILURE;
			goto out;
		}
	}
	data.sender.state = SEND_STATE_IDLE;

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

	/* Pick the parameters for the stream. */
	if (receiver) {
		input_params.device = Pa_GetDefaultInputDevice();
		input_params.channelCount = 1;
		input_params.sampleFormat = paFloat32;
		input_params.suggestedLatency =
			Pa_GetDeviceInfo(input_params.device)->defaultLowInputLatency;
		input_params.hostApiSpecificStreamInfo = NULL;
	}
	if (sender) {
		output_params.device = Pa_GetDefaultOutputDevice();
		output_params.channelCount = 1;
		output_params.sampleFormat = paFloat32;
		output_params.suggestedLatency =
			Pa_GetDeviceInfo(output_params.device)->defaultLowOutputLatency;
		output_params.hostApiSpecificStreamInfo = NULL;
	}

	/* Open a stream and start it. */
	err = Pa_OpenStream(&stream,
			    receiver ? &input_params : NULL,
			    sender ? &output_params : NULL,
			    sample_rate, paFramesPerBufferUnspecified,
			    paClipOff, sofi_callback, &data);
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

	/* Run the sender and/or receiver. */
	if (sender && receiver) {
		void *retval;
		ret = pthread_create(&sender_thread, NULL, sender_start,
				     &data.sender.buffer);
		if (ret) {
			errno = ret;
			perror("pthread_create");
			status = EXIT_FAILURE;
			goto stop_stream;
		}
		ret = receiver_loop(&data.receiver.buffer, window_buffer);
		if (ret)
			status = EXIT_FAILURE;
		pthread_cancel(sender_thread);
		pthread_join(sender_thread, &retval);
		if (retval)
			status = EXIT_FAILURE;
	} else if (sender) {
		ret = sender_loop(&data.sender.buffer);
		if (ret)
			status = EXIT_FAILURE;
	} else if (receiver) {
		ret = receiver_loop(&data.receiver.buffer, window_buffer);
		if (ret)
			status = EXIT_FAILURE;
	}

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
	free(window_buffer);
	return status;
}
