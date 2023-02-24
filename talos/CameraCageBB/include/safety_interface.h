#ifndef SAFETY_INTERFACE_H
#define SAFETY_INTERFACE_H

#include "safety/safety.h"

// NOTE: If adding fault IDs make sure to update the fault_string_list as well

//      FAULT_WATCHDOG_RESET      0
#define FAULT_CAN_INTERNAL_ERROR    1
#define FAULT_CAN_RECV_ERROR        2
#define FAULT_ASYNC_I2C_ERROR       3
#define FAULT_DEPTH_INIT_ERROR      4
#define FAULT_DEPTH_ERROR           5

static const char * const fault_string_list[] = {
    "FAULT_WATCHDOG_RESET",
    "FAULT_CAN_INTERNAL_ERROR",
    "FAULT_CAN_RECV_ERROR",
    "FAULT_ASYNC_I2C_ERROR",
    "FAULT_DEPTH_INIT_ERROR",
    "FAULT_DEPTH_ERROR", 
};


// If no kill switches defined, set NUM_KILL_SWITCHES = 0
enum kill_switch {
    ROS_KILL_SWITCH = 0,
    // Used to automatically calculate number of kill switches
    // This must be the last enum
    NUM_KILL_SWITCHES
};

#endif