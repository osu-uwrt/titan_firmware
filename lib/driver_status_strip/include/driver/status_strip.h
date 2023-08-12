#ifndef DRIVER__STATUS_STRIP_H_
#define DRIVER__STATUS_STRIP_H_

#include "hardware/pio.h"

/**
 * @file driver/status_strip.h
 *
 * @brief Driver to allow control of the WS2812B Neopixel strips at the top of the mark 2 electronics cages to show
 * status information.
 *
 * This driver installs a background timer to handle the automatic refreshing of the LEDs. This exposes the required
 * functions to show the required visual behaviors on the LED strip. This driver is not designed to provide flexibility,
 * but instead should be modified to add any additional features.
 */

/**
 * @brief Possible operation modes for the LED strip.
 */
enum status_strip_mode {
    STATUS_STRIP_MODE_SOLID,
    STATUS_STRIP_MODE_SLOW_FLASH,
    STATUS_STRIP_MODE_FAST_FLASH,
    STATUS_STRIP_MODE_BREATH
};

/**
 * @brief Initializes the neopixel status LED strip library.
 * The strip will briefly flash a startup sequence after this is called.
 *
 * @param pio PIO machine to assign
 * @param sm PIO state machine to assign
 * @param pin Pin connected to pixels
 * @param first_pixel_is_rear True if the firest neopixel is in the rear, false if first in the front
 */
void status_strip_init(PIO pio, uint sm, uint pin, bool first_pixel_is_rear);

/**
 * @brief Sets the status strip to the following color command
 *
 * @param mode The LED strip mode
 * @param red Red color component (0-255)
 * @param green Green color component (0-255)
 * @param blue Blue color component (0-255)
 */
void status_strip_set(enum status_strip_mode mode, uint8_t red, uint8_t green, uint8_t blue);

/**
 * @brief Temporarily flashes the strip with the provided color.
 * Used to alert of a status change while preserving the color command (such as kill switch insertion/removal).
 *
 * @param red Red color component (0-255)
 * @param green Green color component (0-255)
 * @param blue Blue color component (0-255)
 */
void status_strip_status_flash(uint8_t red, uint8_t green, uint8_t blue);

/**
 * @brief Clears the current command and sets the status strip to off.
 */
static inline void status_strip_clear(void) {
    status_strip_set(STATUS_STRIP_MODE_SOLID, 0, 0, 0);
}

/**
 * @brief Enables strip output.
 * @note This is implied by default, and is only needed to re-enable after disabling the status strip.
 */
void status_strip_enable(void);

/**
 * @brief Disables all strip output, supressing set, temporary flash, and startup output.
 * This preserves the current state, and only prevents updates from outputting data to the LEDs.
 * The strip can be re-enabled using `status_strip_enable`
 */
void status_strip_disable(void);

#endif
