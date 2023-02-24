#ifndef _ROBOT__PUDDLES_H
#define _ROBOT__PUDDLES_H

#ifdef UWRT_ROBOT_DEFINED
#error Multiple robot types defined
#endif
#define UWRT_ROBOT_DEFINED

// ROS Namespace
#define ROBOT_NAMESPACE "puddles"

// Address Definitions
#define MICRO_IP {192, 168, 1, 13}
#define MICRO_PORT 1337
#define ROBOT_COMPUTER_IP   {192, 168, 1, 22}
#define ROBOT_COMPUTER_UROS_PORT 8888

// Configuration Options
#define HW_USE_DSHOT 0
#define HW_USE_PWM   1

#endif