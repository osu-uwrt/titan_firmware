#ifndef SAFETY_INTERFACE_H
#define SAFETY_INTERFACE_H

#include "safety/safety.h"

// NOTE: If adding fault IDs make sure to update the fault_string_list as well

//      FAULT_WATCHDOG_RESET      0
//      FAULT_WATCHDOG_WARNING    1
#define FAULT_CAN_INTERNAL_ERROR  2
#define FAULT_CAN_RECV_ERROR      3
#define FAULT_ROS_ERROR           4
#define FAULT_TIMER_MISSED        5
#define FAULT_ROS_BAD_COMMAND     6

static const char * const fault_string_list[] = {
    "FAULT_WATCHDOG_RESET",
    "FAULT_WATCHDOG_WARNING",
    "FAULT_CAN_INTERNAL_ERROR",
    "FAULT_CAN_RECV_ERROR",
    "FAULT_ROS_ERROR",
    "FAULT_TIMER_MISSED",
    "FAULT_ROS_BAD_COMMAND"
};

extern bool safety_interface_kill_switch_refreshed;

#endif