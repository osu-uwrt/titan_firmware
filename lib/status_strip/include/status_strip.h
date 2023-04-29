#ifndef STATUS_STRIP_H
#define STATUS_STRIP_H

#include "hardware/pio.h"

enum status_strip_mode {
    STATUS_STRIP_MODE_SOLID,
    STATUS_STRIP_MODE_SLOW_FLASH,
    STATUS_STRIP_MODE_FAST_FLASH,
    STATUS_STRIP_MODE_BREATH
};

void status_strip_init(PIO pio, uint sm, uint pin, bool first_pixel_is_rear);

void status_strip_set(enum status_strip_mode mode, uint8_t red, uint8_t green, uint8_t blue);

void status_strip_flash_front(uint8_t red, uint8_t green, uint8_t blue);

static inline void status_strip_clear(void) {
    status_strip_set(STATUS_STRIP_MODE_SOLID, 0, 0, 0);
}
#endif