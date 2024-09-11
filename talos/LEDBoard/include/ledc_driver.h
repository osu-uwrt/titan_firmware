#ifndef LEDC_DRIVER_H_
#define LEDC_DRIVER_H_

#include "pico/types.h"

/**
 * @brief Possible operation modes for the LEDs.
 */
enum status_mode { MODE_SOLID, MODE_SLOW_FLASH, MODE_FAST_FLASH, MODE_BREATH };

void led_set(enum status_mode mode, uint8_t red, uint8_t green, uint8_t blue);
void led_status_flash(uint8_t red, uint8_t green, uint8_t blue);
void led_enable(void);
void led_disable(void);

void ledc_init();

#endif
