#ifndef LEDS_H
#define LEDS_H

#include "opt_prototypes.h"

#include <stdint.h>

#define LED_COUNT (6)

void set_rgb(uint8_t address, uint8_t r, uint8_t g, uint8_t b);
void set_hsv(uint8_t address, uint16_t h, uint8_t s, uint8_t v);
void update_leds(opt_callback_t cb);
void lock_leds(void);
void unlock_leds(void);

#endif // LEDS_H
