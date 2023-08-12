#ifndef SAFETY_INTERFACE_H
#define SAFETY_INTERFACE_H

#include "titan/safety.h"

// NOTE: If adding fault IDs make sure to update the fault_string_list as well

//      FAULT_WATCHDOG_RESET      0
//      FAULT_WATCHDOG_WARNING    1
#define FAULT_CAN_INTERNAL_ERROR  2
#define FAULT_ROS_ERROR           3
#define FAULT_TIMER_MISSED        4
#define FAULT_ACTUATOR_FAILURE    5     // Raised if an actuator command could not successfully due to a hardware/firmware issue
#define FAULT_ACTUATOR_UNPLUGGED  6     // Raised if an actuator is unplugged, cleared when all actuators are plugged in

static const char * const fault_string_list[] = {
    "FAULT_WATCHDOG_RESET",
    "FAULT_WATCHDOG_WARNING",
    "FAULT_CAN_INTERNAL_ERROR",
    "FAULT_ROS_ERROR",
    "FAULT_TIMER_MISSED",
    "FAULT_ACTUATOR_FAILURE",
    "FAULT_ACTUATOR_UNPLUGGED"
};


// If no kill switches defined, set NUM_KILL_SWITCHES = 0
enum kill_switch {
    ROS_KILL_SWITCH = 0,
    // Used to automatically calculate number of kill switches
    // This must be the last enum
    NUM_KILL_SWITCHES
};

#endif
