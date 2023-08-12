#ifndef DISPLAY_H
#define DISPLAY_H

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
