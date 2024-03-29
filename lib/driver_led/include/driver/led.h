#ifndef DRIVER__LED_H_
#define DRIVER__LED_H_

#include <stdbool.h>

/**
 * @file driver/led.h
 *
 * @brief Driver for the RGB status LED on each of Mark 2 boards.
 *
 */

/**
 * @brief Initialize the RGB status LED
 */
void led_init();

/**
 * @brief Updates the LED pins
 *
 * Required to ensure that the LED toggles if a fault is present
 * Call this periodically to refresh the LED for toggling between fault and the other color
 */
void led_update_pins();

/**
 * @brief Set LED status to reflect if a fault is present in safety
 *
 * @param value True if fault present in safety
 */
void led_fault_set(bool value);

/**
 * @brief Set LED to reflect if the network is online (CAN, eth, etc.)
 *
 * @param value True if network is online
 */
void led_network_online_set(bool value);

/**
 * @brief Set LED to reflect is the ROS agent is connected
 *
 * @param value True if the ros agent is connected
 */
void led_ros_connected_set(bool value);

/**
 * @brief Set LED to refrlect if the kill switch is inserted
 *
 * @param value True if the kill switch is inserted
 */
void led_killswitch_set(bool value);

#endif
