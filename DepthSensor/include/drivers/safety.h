#ifndef _SAFETY_H
#define _SAFETY_H

#include "safety/safety.h"

// NOTE: If adding fault IDs make sure to update the fault_string_list as well

//      FAULT_WATCHDOG_RESET   0
#define FAULT_ROS_SOFT_FAIL    1
#define FAULT_ROS_BAD_COMMAND  2
#define FAULT_ASYNC_I2C_ERROR  3
#define FAULT_DEPTH_INIT_ERROR 4
#define FAULT_DEPTH_ERROR      5
static const char * const fault_string_list[] = {
    "FAULT_WATCHDOG_RESET",
    "FAULT_ROS_SOFT_FAIL",
    "FAULT_ROS_BAD_COMMAND",
    "FAULT_ASYNC_I2C_ERROR",
    "FAULT_DEPTH_INIT_ERROR",
    "FAULT_DEPTH_ERROR"
};

#endif