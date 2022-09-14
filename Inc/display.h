
#ifndef DISPLAY_H
#define DISPLAY_H

#include <stdint.h>

void display_init(void);
void display_set_hsv(uint8_t led, uint8_t h, uint8_t s, uint8_t v);
void display_set_rgb(uint8_t led, uint8_t r, uint8_t g, uint8_t b);

void display_set_fast_rgb(uint8_t start, uint8_t count, uint8_t* values);
void display_set_fast_hsv(uint8_t start, uint8_t count, uint8_t* values);

void display_clear(void);
void display_request_update(void);

#endif // DISPLAY_H
