#ifndef SOFI_H
#define SOFI_H

#include <stdbool.h>
#include <stdint.h>

struct sofi_packet {
	uint8_t len;
	char payload[UINT8_MAX];
};

struct sofi_init_parameters {
	/* The capture/output sample rate. */
	float sample_rate;
	/* Number of symbols per second. */
	float baud;
	/* Factor of symbol length to use for detecting a carrier wave. */
	float recv_window_factor;
	/* Size of a symbol in bits (must be 1, 2, 4, or 8). */
	int symbol_width;
	/* 1 << symbol_width frequencies in Hz to use as the symbols. */
	float symbol_freqs[1 << 8];
	/* Run the sender/receiver. */
	bool sender, receiver;
	/* Level of debugging messages to print. */
	int debug_level;
};

#define DEFAULT_SOFI_INIT_PARAMS {	\
	.sample_rate = 192000.f,	\
	.baud = 1200.f,			\
	.recv_window_factor = 0.2f,	\
	.symbol_width = 2,		\
	.symbol_freqs = {2400.f, 1200.f, 4800.f, 3600.f}, \
	.sender = true,			\
	.receiver = true,		\
	.debug_level = 0,		\
}

/**
 * sofi_init() - initialize the So-Fi library
 * @params: instance parameters
 *
 * Return: 0 on success, -1 on error.
 */
int sofi_init(const struct sofi_init_parameters *params);

/**
 * sofi_destroy() - free the resources used by the So-Fi library
 *
 * Any outstanding packets will be transmitted before this returns.
 */
void sofi_destroy(void);

/**
 * sofi_send() - send a packet over So-Fi
 *
 * This will block until the packet is queued, but it will not wait for it to be
 * transmitted.
 */
void sofi_send(const struct sofi_packet *packet);

/**
 * sofi_recv() - receive a packet over So-Fi
 *
 * This will block until a packet is available.
 */
void sofi_recv(struct sofi_packet *packet);

#endif /* SOFI_H */
