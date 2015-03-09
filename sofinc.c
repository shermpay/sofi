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
#include <stddef.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "portaudio.h"
#include "pa_ringbuffer.h"

#define M_PI 3.14159265359f

static const char *progname = "sofinc";

static int debug_level;
void debug_printf(int v, const char *format, ...)
{
	va_list ap;

	if (debug_level >= v) {
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
#define SENDER_BUFFER_SIZE 2UL /* 2 packets. */
#define RECEIVER_BUFFER_SIZE (1UL << 20) /* 1M samples. */

static long sample_rate = 192000L;
static float baud = 1000.f;
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

/* Frequencies in Hz for each symbol value. */
static float symbol_freqs[1 << 8] = {2200.f, 1200.f};

static inline int num_symbols(void)
{
	return 1 << symbol_width;
}

static inline unsigned int symbols_per_byte(void)
{
	return CHAR_BIT / symbol_width;
}

/* So-Fi state. */

#define MAX_PACKET_LENGTH 16
struct sofi_packet {
	uint8_t len;
	char payload[UINT8_MAX + 1];
};

#define MAX_MSG_SYMBOLS ((offsetof(struct sofi_packet, payload) + MAX_PACKET_LENGTH) * 8)
struct raw_message {
	size_t len;
	unsigned char symbols[MAX_MSG_SYMBOLS];
};

enum sender_state {
	SEND_STATE_IDLE,
	SEND_STATE_TRANSMITTING,
	SEND_STATE_INTERPACKET_GAP,
};

enum receiver_state {
	RECV_STATE_LISTEN,
	RECV_STATE_DEMODULATE,
};

struct callback_data {
	struct sender_callback_data {
		enum sender_state state;
		PaUtilRingBuffer buffer;
		struct raw_message *msg;
		size_t index;
		unsigned char symbol;
		unsigned long frame;
		float phase;
	} sender;
	struct receiver_callback_data {
		PaUtilRingBuffer buffer;
	} receiver;
};

static void sender_callback(void *output_buffer,
			    unsigned long frames_per_buffer,
			    struct sender_callback_data *data)
{
	ring_buffer_size_t ret;
	float *out = output_buffer;
	float frequency;
	void *data1, *data2;
	ring_buffer_size_t size1, size2;
	bool first = false;

	for (unsigned long i = 0; i < frames_per_buffer; i++) {
		switch (data->state) {
		case SEND_STATE_IDLE:
			ret = PaUtil_GetRingBufferReadRegions(&data->buffer, 1,
							      &data1, &size1,
							      &data2, &size2);
			if (ret == 0) {
				out[i] = 0.f;
				break;
			}
			assert(size1 == 1);
			assert(size2 == 0);

			data->msg = data1;
			data->index = 0;
			data->state = SEND_STATE_TRANSMITTING;
			first = true;
			/* Fallthrough. */
		case SEND_STATE_TRANSMITTING:
			if (first || ++data->frame >= sample_rate / baud) {
				if (data->index >= data->msg->len) {
					PaUtil_AdvanceRingBufferReadIndex(&data->buffer, 1);
					data->state = SEND_STATE_INTERPACKET_GAP;
					data->frame = 0;
					out[i] = 0.f;
					break;
				}
				data->symbol = data->msg->symbols[data->index++];
				data->frame = 0;
			}

			out[i] = sinf(data->phase);
			frequency = symbol_freqs[data->symbol];
			data->phase += (2.f * M_PI * frequency) / sample_rate;
			while (data->phase >= 2.f * M_PI)
				data->phase -= 2.f * M_PI;
			first = false;
			break;
		case SEND_STATE_INTERPACKET_GAP:
			out[i] = 0.f;
			if (++data->frame >= interpacket_gap() * sample_rate)
				data->state = SEND_STATE_IDLE;
			break;
		}
	}
}

static void receiver_callback(const void *input_buffer,
			      unsigned long frames_per_buffer,
			      struct receiver_callback_data *data)
{
	ring_buffer_size_t ret;

	ret = PaUtil_GetRingBufferWriteAvailable(&data->buffer);
	assert((unsigned long)ret >= frames_per_buffer);
	ret = PaUtil_WriteRingBuffer(&data->buffer, input_buffer, frames_per_buffer);
	assert((unsigned long)ret == frames_per_buffer);
}

static int sofi_callback(const void *input_buffer, void *output_buffer,
			 unsigned long frames_per_buffer,
			 const PaStreamCallbackTimeInfo *time_info,
			 PaStreamCallbackFlags status_flags, void *arg)
{
	struct callback_data *data = arg;
	(void)time_info;
	(void)status_flags;

	if (output_buffer)
		sender_callback(output_buffer, frames_per_buffer, &data->sender);
	if (input_buffer && data->sender.state == SEND_STATE_IDLE)
		receiver_callback(input_buffer, frames_per_buffer, &data->receiver);

	return paContinue;
}

static int sender_loop(PaUtilRingBuffer *buffer)
{
	struct sofi_packet packet;
	struct raw_message msg;

	for (;;) {
		packet.len = fread(packet.payload, 1, MAX_PACKET_LENGTH, stdin);
		if (packet.len == 0)
			break;

		msg.len = 0;
		for (size_t i = 0; i < sizeof(packet.len) + packet.len; i++) {
			unsigned char c = ((unsigned char *)&packet)[i];
			for (unsigned int j = 0; j < symbols_per_byte(); j++) {
				msg.symbols[msg.len++] = c & ((1 << symbol_width) - 1);
				c >>= symbol_width;
			}
		}
		while (PaUtil_WriteRingBuffer(buffer, &msg, 1) < 1)
			Pa_Sleep(CHAR_BIT * 1000.f / baud);
	}

	/* Wait for any outstanding output to be sent. */
	while (PaUtil_GetRingBufferReadAvailable(buffer) > 0)
		Pa_Sleep(CHAR_BIT * 1000.f / baud);

	if (ferror(stdin) && errno != EINTR) {
		perror("fread");
		return -1;
	}
	return 0;
}

static void *sender_start(void *arg)
{
	return (void *)(uintptr_t)sender_loop(arg);
}

static int print_message(const struct raw_message *msg)
{
	struct sofi_packet packet;
	unsigned char c;

	memset(&packet, 0, sizeof(packet));
	for (size_t i = 0; i < msg->len; i++) {
		c = msg->symbols[i] << ((i % symbols_per_byte()) * symbol_width);
		((unsigned char *)&packet)[i / symbols_per_byte()] |= c;
	}

	if (debug_level < 1) {
		fwrite(packet.payload, 1, packet.len, stdout);
	} else {
		if (packet.len == 0 && debug_level < 2)
			return 0;
		printf("sofi_packet = {\n");
		printf("\t.len = %" PRIu8 "\n", packet.len);
		printf("\t.payload = \"");
		for (unsigned i = 0; i < packet.len; i++) {
			char c = packet.payload[i];
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
	int signum;
	enum receiver_state state = RECV_STATE_LISTEN;
	ring_buffer_size_t ring_ret;
	struct raw_message msg;
	int symbol;
	float max_strength;

	while (!(signum = signal_received)) {
		int window_size;

		if (state == RECV_STATE_LISTEN)
			window_size = receiver_window();
		else
			window_size = (int)((float)sample_rate / baud);

		if (PaUtil_GetRingBufferReadAvailable(buffer) < window_size) {
			Pa_Sleep(1000.f * window_size / sample_rate);
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

			debug_printf(3, "%s%f", (i > 0) ? ", " : "", strength);
		}
		debug_printf(3, "] = %d\n", symbol);

		switch (state) {
		case RECV_STATE_LISTEN:
			if (symbol != -1) {
				memset(&msg, 0, sizeof(msg));
				state = RECV_STATE_DEMODULATE;
				debug_printf(2, "-> DEMODULATE\n");
			}
			break;
		case RECV_STATE_DEMODULATE:
			if (symbol == -1) {
				if (print_message(&msg))
					return -1;
				debug_printf(2, "-> LISTEN\n");
				state = RECV_STATE_LISTEN;
				break;
			}

			if (msg.len < MAX_MSG_SYMBOLS)
				msg.symbols[msg.len++] = symbol;
			break;
		}
	}

	fprintf(stderr, "got %d; exiting\n", signum);
	return 0;
}

static void usage(bool error)
{
	fprintf(error ? stderr : stdout,
		"Usage: %s [OPTION]...\n"
		"Transmit data over sound, reading from standard input and writing to standard\n"
		"output.\n"
		"\n"
		"Communication direction:\n"
		"  -R, --receiver                     run the receiver (enabled by default unless\n"
		"                                     --sender is given)\n"
		"  -S, --sender                       run the sender (enabled by default unless\n"
		"                                     --receiver is given)\n"
		"\n"
		"Transmission parameters:\n"
		"  -s, --sample-rate=SAMPLE_RATE      set up the streams at SAMPLE_RATE\n"
		"  -f, --frequencies=FREQ0,FREQ1,...  use the given frequencies for symbols,\n"
		"                                     with 2, 4, 16, or 256 frequencies for a\n"
		"                                     symbol width of 1, 2, 4, or 8, respectively\n"
		"  -w, --window=WINDOW_FACTOR         use a window of size WINDOW_FACTOR times\n"
		"                                     the symbol duration time to detect a carrier\n"
		"                                     wave\n"
		"  -b, --baud=BAUD                    run at BAUD symbols per second\n"
		"\n"
		"Miscellaneous:\n"
		"  -d                                 increase the debug level by one\n"
		"  --debug-level=DEBUG_LEVEL          set the debug level to DEBUG_LEVEL\n"
		"  -h, --help                         display this help text and exit\n"
		, progname);
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
	PaStreamParameters input_params, output_params;
	bool sender = false, receiver = false;

	if (argc > 0)
		progname = argv[0];
	for (;;) {
		static struct option longopts[] = {
			{"receiver",	no_argument,		NULL,	'R'},
			{"sender",	no_argument,		NULL,	'S'},
			{"sample-rate",	required_argument,	NULL,	's'},
			{"frequencies",	required_argument,	NULL,	'f'},
			{"window",	required_argument,	NULL,	'w'},
			{"baud",	required_argument,	NULL,	'b'},
			{"debug-level",	required_argument,	NULL,	'd'},
			{"help",	no_argument,		NULL,	'h'},
		};
		int opt;
		int longindex;
		char *end;
		float freq;
		int i;

		opt = getopt_long(argc, argv, "RSb:f:s:w:dh",
				  longopts, &longindex);
		if (opt == -1)
			break;

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
		case 'd':
			if (optarg)
				debug_level = atoi(optarg);
			else
				debug_level++;
			break;
		case 'h':
			usage(false);
		default:
			usage(true);
		}
	}
	if (!sender && !receiver)
		sender = receiver = true;
	if (debug_level > 0) {
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
		sender_buffer_ptr = malloc(SENDER_BUFFER_SIZE * sizeof(struct raw_message));
		if (!sender_buffer_ptr) {
			perror("malloc");
			status = EXIT_FAILURE;
			goto out;
		}
		PaUtil_InitializeRingBuffer(&data.sender.buffer,
					    sizeof(struct raw_message),
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
