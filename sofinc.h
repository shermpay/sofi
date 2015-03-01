#ifndef FSK_H
#define FSK_H

#include <inttypes.h>
#include <limits.h>

#define M_PI 3.14159265359f

#define BAUD 100

#define INTERPACKET_GAP (2.f / BAUD)

#define MAX_PACKET_LENGTH 10

struct sofi_packet {
	uint8_t len;
	char payload[UINT8_MAX];
};

/* Size of a symbol in bits (must be 1, 2, 4, or 8). */
#define SYMBOL_WIDTH 1
#define NUM_SYMBOLS (1 << SYMBOL_WIDTH)
#define SYMBOLS_PER_BYTE (CHAR_BIT / SYMBOL_WIDTH)

/* Frequencies in Hz for each symbol value. */
static const float symbol_freqs[NUM_SYMBOLS] = {
	2200.f,		/* 0 */
	1200.f,		/* 1 */
};

static inline unsigned int symbol_from_byte(unsigned char c, unsigned int i)
{
	unsigned int mask = ((1 << SYMBOL_WIDTH) - 1) << (SYMBOL_WIDTH * i);
	return (c & mask) >> (SYMBOL_WIDTH * i);
}

static inline unsigned char bits_from_symbol(unsigned int s, unsigned int i)
{
	return s << (SYMBOL_WIDTH * i);
}

#endif /* FSK_H */
