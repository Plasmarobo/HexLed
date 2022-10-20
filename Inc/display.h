
#ifndef DISPLAY_H
#define DISPLAY_H

#include <stdint.h>

typedef enum {
    PATTERN_NONE = 0,
    PATTERN_SOLID,
    PATTERN_BREATHE,
    PATTERN_HEARTBEAT,
    PATTERN_BLINK,
    PATTERN_SOS,
    PATTERN_MAX,
} pattern_t;

void display_init(void);
void display_set_hsv(uint8_t led, uint8_t h, uint8_t s, uint8_t v);
void display_set_rgb(uint8_t led, uint8_t r, uint8_t g, uint8_t b);

void display_set_fast_rgb(uint8_t start, uint8_t count, uint8_t* values);
void display_set_fast_hsv(uint8_t start, uint8_t count, uint8_t* values);

void display_clear(void);

void set_pattern_rgb(pattern_t pattern, uint8_t led, uint8_t r, uint8_t g, uint8_t b);
void clear_pattern(uint8_t led);

#endif // DISPLAY_H
