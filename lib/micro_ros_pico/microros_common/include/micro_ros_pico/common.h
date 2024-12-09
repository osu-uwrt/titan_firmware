#ifndef MICRO_ROS_PICO__COMMON_H_
#define MICRO_ROS_PICO__COMMON_H_

/**
 * @brief Set up error handling handler to report any RMW errors
 * Makes life a *lot* easier, as long as debug logging is enabled
 */
void micro_ros_init_error_handling(void);

#endif
