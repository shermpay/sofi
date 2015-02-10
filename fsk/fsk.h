#ifndef FSK_H
#define FSK_H

#include <inttypes.h>

#define BAUD 10
#define ZERO_FREQ 2200.f
#define ONE_FREQ 1200.f

#define INTERPACKET_GAP (2.f / BAUD)

struct sofi_packet {
	uint8_t len;
	char payload[UINT8_MAX];
};

#endif /* FSK_H */
