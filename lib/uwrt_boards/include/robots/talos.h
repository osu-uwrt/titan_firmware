#ifndef _ROBOT__TALOS_H
#define _ROBOT__TALOS_H

#ifdef UWRT_ROBOT_DEFINED
#error Multiple robot types defined
#endif
#define UWRT_ROBOT_DEFINED

// ROS Namespace
#define ROBOT_NAMESPACE "talos"

// CAN Bus Definitions
// Note that each can bus is defined by `BUS_NAME`_`PARAMETER_NAME`
// The following parameters are required: ENABLE_FD, RATE, and FD_RATE (if ENABLE_FD is 1)
#define INTERNAL_CAN_ENABLE_FD  1
#define INTERNAL_CAN_RATE       1000000
#define INTERNAL_CAN_FD_RATE    5000000
#define INTERNAL_CAN_ID         1

#define EXTERNAL_CAN_ENABLE_FD  0
#define EXTERNAL_CAN_RATE       250000
#define EXTERNAL_CAN_ID         2

// Configuration Options
#define HW_USE_DSHOT 0
#define HW_USE_PWM   1

#endif