
#include "leds.h"

#include "FreeRTOS.h"
#include "fast_hsv2rgb.h"
#include "semphr.h"
#include "spi.h"

#include <assert.h>
#include <stdint.h>
#include <string.h>

// WS2812 requires 1.25us timing +- 600ns
// HIGH pattern: 0.35us HIGH, 0.7us LOW
// LOW pattern: 0.8us HIGH, 0.6us LOW
// RST pattern: 1.25us low

// Our SPI is clocked at 3.125MHz
// Each bit has a period of 320ns
// High = 100
// Low = 110
// Rst = 000
// Need 50us of 0 to reset = 157 bits = 20 bytes

#define BITS_PER_BIT (3)
#define RST_BYTES (21)
// One byte per color channel
#define BYTES_PER_LED (BITS_PER_BIT * 3)
#define CODEPOINT_LENGTH (RST_BYTES + (BYTES_PER_LED * LED_COUNT))

/*
 * Pointer swapping:
 * 	sext.	r g b	r<>b	g<>b	r <> g	result
 *	0 0 0	v u c			!u v c	u v c
 *	0 0 1	d v c				d v c
 *	0 1 0	c v u	u v c			u v c
 *	0 1 1	c d v	v d c		d v c	d v c
 *	1 0 0	u c v		u v c		u v c
 *	1 0 1	v c d		v d c	d v c	d v c
 *
 * if(sextant & 2)
 * 	r <-> b
 *
 * if(sextant & 4)
 * 	g <-> b
 *
 * if(!(sextant & 6) {
 * 	if(!(sextant & 1))
 * 		r <-> g
 * } else {
 * 	if(sextant & 1)
 * 		r <-> g
 * }
 */
static inline void swapptr(uint8_t** a, uint8_t** b)
{
    uint8_t* tmp = *a;
    *a = *b;
    *b = tmp;
}

static inline void hsv_pointer_swap(uint8_t sextant, uint8_t** r, uint8_t** g, uint8_t** b)
{
    if((sextant) & 2) {
        swapptr((r), (b)); 
    }
    if((sextant) & 4) {
        swapptr((g), (b));
    }
    if(!((sextant) & 6)) {
        if(!((sextant) & 1)) {
            swapptr((r), (g));
        }
    } else {
        if((sextant) & 1) {
            swapptr((r), (g));
        }
    }
}

// Adafruit gamma table

/* I disagree with 6.7.9p11 in this context */
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-W"
static const uint8_t   gamma8[] = {
    0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,
    0,   0,   0,   0,   0,   0,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   2,   2,   2,
    2,   2,   2,   2,   2,   3,   3,   3,   3,   3,   3,   3,   4,   4,   4,   4,   4,   5,   5,   5,   5,   6,
    6,   6,   6,   7,   7,   7,   7,   8,   8,   8,   9,   9,   9,   10,  10,  10,  11,  11,  11,  12,  12,  13,
    13,  13,  14,  14,  15,  15,  16,  16,  17,  17,  18,  18,  19,  19,  20,  20,  21,  21,  22,  22,  23,  24,
    24,  25,  25,  26,  27,  27,  28,  29,  29,  30,  31,  32,  32,  33,  34,  35,  35,  36,  37,  38,  39,  39,
    40,  41,  42,  43,  44,  45,  46,  47,  48,  49,  50,  50,  51,  52,  54,  55,  56,  57,  58,  59,  60,  61,
    62,  63,  64,  66,  67,  68,  69,  70,  72,  73,  74,  75,  77,  78,  79,  81,  82,  83,  85,  86,  87,  89,
    90,  92,  93,  95,  96,  98,  99,  101, 102, 104, 105, 107, 109, 110, 112, 114, 115, 117, 119, 120, 122, 124,
    126, 127, 129, 131, 133, 135, 137, 138, 140, 142, 144, 146, 148, 150, 152, 154, 156, 158, 160, 162, 164, 167,
    169, 171, 173, 175, 177, 180, 182, 184, 186, 189, 191, 193, 196, 198, 200, 203, 205, 208, 210, 213, 215, 218,
    220, 223, 225, 228, 231, 233, 236, 239, 241, 244, 247, 249, 252, 255,
};

static uint8_t codepoints[CODEPOINT_LENGTH] = {0};

const uint32_t LOW = 0x4;
const uint32_t HIGH = 0x6;
const uint32_t RST = 0x0;

static inline uint32_t byte_to_wave(uint8_t b)
{
    uint32_t wave = 0;
    for (uint32_t input_bit = 0; input_bit < 8; ++input_bit)
    {
        if (b & (1 << input_bit))
        {
            // Bit is set, write
            wave |= (HIGH << (input_bit * 3));
        }
        else
        {
            wave |= (LOW << (input_bit * 3));
        }
    }
    return wave;
}

static inline void set_codepoint(uint32_t* offset, uint32_t wave)
{
    // MSB first configuration
    codepoints[(*offset)++] = (wave >> 16) & 0xFF;
    codepoints[(*offset)++] = (wave >> 8) & 0xFF;
    codepoints[(*offset)++] = (wave) & 0xFF;
}

void set_rgb(uint8_t address, uint8_t r, uint8_t g, uint8_t b)
{
    if (address < LED_COUNT)
    {
        uint32_t offset = RST_BYTES + (address * BYTES_PER_LED);
        set_codepoint(&offset, byte_to_wave(gamma8[g]));
        set_codepoint(&offset, byte_to_wave(gamma8[r]));
        set_codepoint(&offset, byte_to_wave(gamma8[b]));
    }
}

// Using fixed point, h is angle in degrees expressed from 0-360
// s and v are on the order of 255
void set_hsv(uint8_t address, uint16_t h, uint8_t s, uint8_t v)
{
    uint8_t r, g, b;
    fast_hsv2rgb_32bit(h, s, v, &r, &g, &b);
    set_rgb(address, r, g, b);
}

void update_leds(opt_callback_t cb)
{
    spi2_send(codepoints, CODEPOINT_LENGTH, cb);
}

