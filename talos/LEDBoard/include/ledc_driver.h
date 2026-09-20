/**
 * @file ledc_driver.h
 * @author Ohio State Underwater Robotic Team (UWRT)
 * @brief Header File for describing the LED drivers for Talos.
 * @version 0.1
 * @date 2026-09-18
 *
 * @copyright Copyright (c) 2026
 *
 */

#ifndef LEDC_DRIVER_H_
#define LEDC_DRIVER_H_

#include "pico/types.h"

/**
 * @brief Possible operation modes for the LEDs.
 */
typedef enum status_mode { MODE_SOLID, MODE_SLOW_FLASH, MODE_FAST_FLASH, MODE_BREATH } status_mode;

/**
 * @brief Opaque Pointer for saving certain states of the RGB values without affecting new updated values.
 */
typedef struct led_rgb_values_saved_t led_rgb_values_saved_t;

/**
 * @brief Opaque Pointer for holding updating runtime LED RGB values.
 */
typedef struct led_rgb_values_runtime_t led_rgb_values_runtime_t;

/**
 * @brief Replaces the current driver mode to the one described in @c mode and updates the @c rgb_target member
 *        variables to be the same as ```red```, ```green```, and ```blue```.
 *
 * @note Sets the local persistent driver mode.
 *
 * @param mode (enum) The new mode that you want to replace the old mode with.
 * @param red The new red color value.
 * @param green The new green color value.
 * @param blue The new blue color value.
 */
void led_set(status_mode mode, uint8_t red, uint8_t green, uint8_t blue);

/**
 * @brief Replaces the current driver mode to flash and updates the @c rgb_target member variables to be the same as
 *        ```red```, ```green```, and ```blue```.
 *
 * @note Flash (on kill switch insertion), not persistent
 *
 * @param red The new red color value.
 * @param green The new green color value.
 * @param blue The new blue color value.
 */
void led_flash(uint8_t red, uint8_t green, uint8_t blue);

/**
 * @brief Replaces the current driver mode to singleton and updates the @c rgb_target member variables to be the same as
 *        ```red```, ```green```, and ```blue```.
 *
 * @note Short flash (on vision detection), not persistent. Singleton = brief flash.
 *
 * @param red The new red color value.
 * @param green The new green color value.
 * @param blue The new blue color value.
 */
void led_singleton(uint8_t red, uint8_t green, uint8_t blue);

/**
 * @brief Clears and resets the current LED Driver to be turned off.
 */
static inline void led_clear(void) {
    led_set(MODE_SOLID, 0, 0, 0);
}

/**
 * @brief Configure, set inital states, start repeating timers. Starts the GPIO and SPI communication between the
 *        Aluminum LED board as well as sets up callbacks for watchdog timers and LED updates.
 */
void ledc_init();

/**
 * @brief Updates the @c is_underwater internal variable to determine LED brightness if under the water.
 *
 * @note Use underwater brightness if below threshold
 *
 * @param depth The current depth relative to the top of the water.
 */
void led_depth_set(float depth);

#endif
