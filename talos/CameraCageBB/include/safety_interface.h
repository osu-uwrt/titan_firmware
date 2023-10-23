#ifndef SAFETY_INTERFACE_H
#define SAFETY_INTERFACE_H

#include "titan/safety.h"

// NOTE: If adding fault IDs make sure to update the fault_string_list as well

//      FAULT_WATCHDOG_RESET      0
//      FAULT_WATCHDOG_WARNING    1
#define FAULT_CAN_INTERNAL_ERROR 2
#define FAULT_ROS_ERROR 3
#define FAULT_TIMER_MISSED 4
#define FAULT_ROS_BAD_COMMAND 5
#define FAULT_DEPTH_INIT_ERROR 6
#define FAULT_DEPTH_ERROR 7
#define FAULT_SHT41_ERROR 8

static const char *const fault_string_list[] = {
    "FAULT_WATCHDOG_RESET", "FAULT_WATCHDOG_WARNING", "FAULT_CAN_INTERNAL_ERROR", "FAULT_ROS_ERROR",
    "FAULT_TIMER_MISSED",   "FAULT_ROS_BAD_COMMAND",  "FAULT_DEPTH_INIT_ERROR",   "FAULT_DEPTH_ERROR",
};

// If no kill switches defined, set NUM_KILL_SWITCHES = 0
enum kill_switch {
    ROS_KILL_SWITCH = 0,
    // Used to automatically calculate number of kill switches
    // This must be the last enum
    NUM_KILL_SWITCHES
};

#endif
