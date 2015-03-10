#include <assert.h>
#include <errno.h>
#include <limits.h>
#include <math.h>
#include <portaudio.h>
#include <pthread.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "sofi.h"
#include "pa_ringbuffer.h"

#define M_PI 3.14159265359f

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

/* Globals, mostly for the sake of cleanup or lifetime. */
static struct callback_data data;
static PaStream *stream;
static void *sender_buffer_ptr;
static void *receiver_buffer_ptr;
static float *window_buffer;
static pthread_t receiver_thread;

struct raw_message {
	size_t len;
	unsigned char symbols[(sizeof(struct sofi_packet) + sizeof(uint32_t)) * 8];
};

/*
 * Receive queue. Received messages are placed here as they are demodulated and
 * removed as the client calls sofi_recv(). Messages will be dropped if they
 * overflow the queue.
 */
#define RECV_QUEUE_CAP 32
static pthread_mutex_t recv_queue_lock = PTHREAD_MUTEX_INITIALIZER;
static pthread_cond_t recv_queue_cond = PTHREAD_COND_INITIALIZER;
static struct raw_message recv_queue[RECV_QUEUE_CAP];
static size_t recv_queue_start, recv_queue_size;

static inline void recv_queue_enqueue(const struct raw_message *msg)
{
	int ret;

	ret = pthread_mutex_lock(&recv_queue_lock);
	assert(ret == 0);

	if (recv_queue_size < RECV_QUEUE_CAP) {
		size_t i = (recv_queue_start + recv_queue_size) % RECV_QUEUE_CAP;
		memcpy(&recv_queue[i], msg, sizeof(struct raw_message));
		recv_queue_size++;
		ret = pthread_cond_signal(&recv_queue_cond);
		assert(ret == 0);
	} else {
		/* The message is dropped if the queue overflows. */
		debug_printf(1, "recv_queue overflow\n");
	}

	ret = pthread_mutex_unlock(&recv_queue_lock);
	assert(ret == 0);
}

static inline void recv_queue_dequeue(struct raw_message *msg)
{
	int ret;

	ret = pthread_mutex_lock(&recv_queue_lock);
	assert(ret == 0);

	while (!recv_queue_size) {
		ret = pthread_cond_wait(&recv_queue_cond, &recv_queue_lock);
		assert(ret == 0);
	}
	memcpy(msg, &recv_queue[recv_queue_start], sizeof(struct raw_message));
	recv_queue_start = (recv_queue_start + 1) % RECV_QUEUE_CAP;
	recv_queue_size--;

	ret = pthread_mutex_unlock(&recv_queue_lock);
	assert(ret == 0);
}

/* Transmission parameters. */
#define SENDER_BUFFER_SIZE 2UL /* 2 packets. */
#define RECEIVER_BUFFER_SIZE (1UL << 20) /* 1M samples. */

static long sample_rate;
static float baud;
static float recv_window_factor;

static inline int receiver_window(void)
{
	return (int)(recv_window_factor / baud * (float)sample_rate);
}

static inline float interpacket_gap(void)
{
	return 2.f / baud;
}

/* Symbol definitions. */

/* Size of a symbol in bits (must be 1, 2, 4, or 8). */
static int symbol_width;

/* Frequencies in Hz for each symbol value. */
static float symbol_freqs[1 << 8];

static inline int num_symbols(void)
{
	return 1 << symbol_width;
}

static inline unsigned int symbols_per_byte(void)
{
	return CHAR_BIT / symbol_width;
}

/* Internal state. */

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
			if (++data->frame >= interpacket_gap() * sample_rate) {
				PaUtil_AdvanceRingBufferReadIndex(&data->buffer, 1);
				data->state = SEND_STATE_IDLE;
			}
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

static void *receiver_loop(void *arg)
{
	PaUtilRingBuffer *buffer = arg;
	enum receiver_state state = RECV_STATE_LISTEN;
	ring_buffer_size_t ring_ret;
	struct raw_message msg;
	int symbol;
	float max_strength;

	for (;; pthread_testcancel()) {
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
				recv_queue_enqueue(&msg);
				debug_printf(2, "-> LISTEN\n");
				state = RECV_STATE_LISTEN;
				break;
			}
			if (msg.len < sizeof(msg.symbols) / sizeof(msg.symbols[0]))
				msg.symbols[msg.len++] = symbol;
			break;
		}
	}
	return (void *)0;
}

int sofi_init(const struct sofi_init_parameters *params)
{
	PaError err;
	int ret;
	PaStreamParameters input_params, output_params;

	sample_rate = params->sample_rate;
	baud = params->baud;
	recv_window_factor = params->recv_window_factor;
	symbol_width = params->symbol_width;
	memcpy(symbol_freqs, params->symbol_freqs,
	       num_symbols() * sizeof(float));
	debug_level = params->debug_level;

	/* Initialize callback data and receiver window buffer. */
	if (params->sender) {
		sender_buffer_ptr = malloc(SENDER_BUFFER_SIZE * sizeof(struct raw_message));
		if (!sender_buffer_ptr) {
			perror("malloc");
			goto err;
		}
		PaUtil_InitializeRingBuffer(&data.sender.buffer,
					    sizeof(struct raw_message),
					    SENDER_BUFFER_SIZE,
					    sender_buffer_ptr);
		data.sender.phase = 0.f;
	}
	if (params->receiver) {
		receiver_buffer_ptr = malloc(RECEIVER_BUFFER_SIZE * sizeof(float));
		if (!receiver_buffer_ptr) {
			perror("malloc");
			goto err;
		}
		PaUtil_InitializeRingBuffer(&data.receiver.buffer,
					    sizeof(float), RECEIVER_BUFFER_SIZE,
					    receiver_buffer_ptr);
		window_buffer = malloc(RECEIVER_BUFFER_SIZE * sizeof(float));
		if (!window_buffer) {
			perror("malloc");
			goto err;
		}
	}

	/* Initialize PortAudio. */
	err = Pa_Initialize();
	if (err != paNoError) {
		fprintf(stderr, "PortAudio: initialization failed: %s\n",
			Pa_GetErrorText(err));
		goto err;
	}

	/* Pick the parameters for the stream. */
	if (params->receiver) {
		input_params.device = Pa_GetDefaultInputDevice();
		input_params.channelCount = 1;
		input_params.sampleFormat = paFloat32;
		input_params.suggestedLatency =
			Pa_GetDeviceInfo(input_params.device)->defaultLowInputLatency;
		input_params.hostApiSpecificStreamInfo = NULL;
	}
	if (params->sender) {
		output_params.device = Pa_GetDefaultOutputDevice();
		output_params.channelCount = 1;
		output_params.sampleFormat = paFloat32;
		output_params.suggestedLatency =
			Pa_GetDeviceInfo(output_params.device)->defaultLowOutputLatency;
		output_params.hostApiSpecificStreamInfo = NULL;
	}

	/* Open a stream and start it. */
	err = Pa_OpenStream(&stream,
			    params->receiver ? &input_params : NULL,
			    params->sender ? &output_params : NULL,
			    sample_rate, paFramesPerBufferUnspecified,
			    paClipOff, sofi_callback, &data);
	if (err != paNoError) {
		fprintf(stderr, "PortAudio: opening stream failed: %s\n",
			Pa_GetErrorText(err));
		goto terminate;
	}
	err = Pa_StartStream(stream);
	if (err != paNoError) {
		fprintf(stderr, "PortAudio: starting stream failed: %s\n",
			Pa_GetErrorText(err));
		goto close_stream;
	}

	/* Start the reciever thread. */
	if (params->receiver) {
		ret = pthread_create(&receiver_thread, NULL, receiver_loop,
				     &data.receiver.buffer);
		if (ret) {
			errno = ret;
			perror("pthread_create");
			goto stop_stream;
		}
	}

	debug_printf(1,
		     "Sample rate:\t%ld Hz\n"
		     "Baud:\t\t%.2f symbols/sec, %d samples, %.2f seconds\n"
		     "Window:\t\t%d samples, %.2f seconds\n",
		     sample_rate,
		     baud, (int)((float)sample_rate / baud), 1.f / baud,
		     receiver_window(), receiver_window() / (float)sample_rate);
	debug_printf(1, "Frequencies:\t");
	for (int i = 0; i < num_symbols(); i++)
		debug_printf(1, "%s%.2f Hz", (i > 0) ? ", " : "", symbol_freqs[i]);
	debug_printf(1, "\n");

	return 0;

stop_stream:
	err = Pa_StopStream(stream);
	if (err != paNoError) {
		fprintf(stderr, "PortAudio: stopping stream failed: %s\n",
			Pa_GetErrorText(err));
	}
close_stream:
	err = Pa_CloseStream(stream);
	if (err != paNoError) {
		fprintf(stderr, "PortAudio: closing stream failed: %s\n",
			Pa_GetErrorText(err));
	}
terminate:
	err = Pa_Terminate();
	if (err != paNoError) {
		fprintf(stderr, "PortAudio: termination failed: %s\n",
			Pa_GetErrorText(err));
	}
err:
	free(sender_buffer_ptr);
	free(receiver_buffer_ptr);
	free(window_buffer);
	return -1;
}

void sofi_destroy(void)
{
	PaError err;

	pthread_cancel(receiver_thread);
	pthread_join(receiver_thread, NULL);

	/*
	 * Wait for any outstanding output to be sent, plus a little extra
	 * because either PortAudio or ALSA can't be trusted.
	 */
	while (PaUtil_GetRingBufferReadAvailable(&data.sender.buffer) > 0)
		Pa_Sleep(CHAR_BIT * 1000.f / baud);
	Pa_Sleep(100);

	err = Pa_StopStream(stream);
	if (err != paNoError) {
		fprintf(stderr, "PortAudio: stopping stream failed: %s\n",
			Pa_GetErrorText(err));
	}
	err = Pa_CloseStream(stream);
	if (err != paNoError) {
		fprintf(stderr, "PortAudio: closing stream failed: %s\n",
			Pa_GetErrorText(err));
	}
	err = Pa_Terminate();
	if (err != paNoError) {
		fprintf(stderr, "PortAudio: termination failed: %s\n",
			Pa_GetErrorText(err));
	}
	free(sender_buffer_ptr);
	free(receiver_buffer_ptr);
	free(window_buffer);
}

static uint32_t crc32(unsigned char *buf, size_t len)
{
	uint32_t tab[256];
	uint32_t val;

	for (int i = 0; i < 256; i++) {
		uint32_t crc = i;
		for (int j = 0; j < 8; j++) {
			if (crc & 1)
				crc = (crc >> 1) ^ UINT32_C(0xedb88320);
			else
				crc >>= 1;
		}
		tab[i] = crc;
	}

	val = ~UINT32_C(0);
	for (size_t i = 0; i < len; i++) {
		int idx = (uint8_t)val ^ (uint8_t)buf[i];
		val = tab[idx] ^ (val >> 8);
	}
	return ~val;
}

void sofi_send(const struct sofi_packet *packet)
{
	struct raw_message msg;
	unsigned char buf[sizeof(*packet) + sizeof(uint32_t)];
	size_t size;
	uint32_t crc;

	size = sizeof(packet->len) + packet->len;
	memcpy(buf, packet, size);
	crc = crc32(buf, size);
	memcpy(buf + size, &crc, sizeof(crc));
	size += sizeof(crc);

	msg.len = 0;
	for (size_t i = 0; i < size; i++) {
		unsigned char c = buf[i];
		for (unsigned int j = 0; j < symbols_per_byte(); j++) {
			msg.symbols[msg.len++] = c & ((1 << symbol_width) - 1);
			c >>= symbol_width;
		}
	}
	while (PaUtil_WriteRingBuffer(&data.sender.buffer, &msg, 1) < 1)
		Pa_Sleep(CHAR_BIT * 1000.f / baud);
}

void sofi_recv(struct sofi_packet *packet)
{
	struct raw_message msg;
	unsigned char buf[sizeof(*packet) + sizeof(uint32_t)];
	uint8_t len;
	uint32_t crc1, crc2;

	for (;;) {
		recv_queue_dequeue(&msg);
		memset(buf, 0, sizeof(buf));
		for (size_t i = 0; i < msg.len; i++) {
			unsigned char c =
				msg.symbols[i] << ((i % symbols_per_byte()) * symbol_width);
			if (i / symbols_per_byte() < sizeof(buf))
				buf[i / symbols_per_byte()] |= c;
		}
		memcpy(&len, buf, sizeof(len));
		memcpy(&crc1, buf + sizeof(len) + len, sizeof(crc1));
		crc2 = crc32(buf, sizeof(len) + len);
		if (crc1 == crc2) {
			memcpy(packet, buf, sizeof(len) + len);
			break;
		} else {
			debug_printf(2, "dropped corrupt packet\n");
		}
	}
}
