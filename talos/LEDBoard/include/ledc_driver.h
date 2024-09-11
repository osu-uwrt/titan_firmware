#ifndef LEDC_DRIVER_H_
#define LEDC_DRIVER_H_

/**
 * @brief Possible operation modes for the LEDs.
 */
enum status_mode { MODE_SOLID, MODE_SLOW_FLASH, MODE_FAST_FLASH, MODE_BREATH };

void ledc_init();

#endif
