#ifndef SOFI_H
#define SOFI_H

#include <stdbool.h>
#include <stdint.h>

struct sofi_packet {
	uint8_t len;
	char payload[UINT8_MAX];
};

/**
 * sofi_init() - initialize the So-Fi library
 *
 * Return: 0 on success, -1 on error.
 */
int sofi_init(bool sender, bool receiver);

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
