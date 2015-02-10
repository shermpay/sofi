#ifndef FSK_H
#define FSK_H

#include <inttypes.h>

#define BAUD 50
#define ZERO_FREQ 2200.f
#define ONE_FREQ 1200.f

#define INTERPACKET_GAP (2.f / BAUD)

#define MAX_PACKET_LENGTH 10

struct sofi_packet {
	uint8_t len;
	char payload[UINT8_MAX];
};

#endif /* FSK_H */
