#ifndef _ROBOT__TEMPEST_H
#define _ROBOT__TEMPEST_H

#ifdef UWRT_ROBOT_DEFINED
#error Multiple robot types defined
#endif
#define UWRT_ROBOT_DEFINED

// ROS Namespace
#define ROBOT_NAMESPACE "tempest"

// Address Definitions
#define ROBOT_COMPUTER_IP   {192, 168, 1, 22}
#define ROBOT_COMPUTER_UROS_PORT 8888

// Configuration Options
#define HW_USE_DSHOT 0
#define HW_USE_PWM   1

#endif