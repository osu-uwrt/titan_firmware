#ifndef DISPLAY_H
#define DISPLAY_H
#include "pico/time.h"

#include <stdbool.h>
#include <stdint.h>
extern absolute_time_t display_poweroff_time;

/**
 * @brief Initialize LCD screen
 *
 */
void display_init(void);

/**
 * @brief Show battery stats on the display
 *
 * @param serial Battery serial number
 * @param soc Battery current SOC
 * @param voltage Battery current voltage
 */
void display_show_stats(unsigned int serial, unsigned int soc, float voltage);

void display_show_menu(uint8_t op_hl);

void display_show_option(unsigned int serial, uint8_t op_hl, bool dsg_mode);

// void display_show_soc(unsigned int serial, uint8_t soc, bool dsg_mode);

// void display_show_voltage(unsigned int serial, float voltage, bool dsg_mode);

// void display_show_current(unsigned int serial, float current, bool dsg_mode);

// void display_show_remain_time(unsigned int serial, uint16_t remain_time, bool dsg_mode);

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
