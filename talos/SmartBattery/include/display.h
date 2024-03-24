#ifndef DISPLAY_H
#define DISPLAY_H
#include "pico/time.h"

#include <stdbool.h>
#include <stdint.h>
/**
 * @brief Initialize LCD screen
 *
 */
void display_init(void);

/**
 * @brief Tick screen update which need to occur within display
 */
void display_tick(uint16_t serial);

/**
 * @brief Show ROS connections screen
 */
void display_show_ros_connect(void);

/**
 * @brief Show ROS disconnect screen
 */
void display_show_ros_disconnect(void);

/**
 * @brief Check if display needs to be powered off after set timeout, and powers off if so.
 */
void display_check_poweroff(void);

#endif
