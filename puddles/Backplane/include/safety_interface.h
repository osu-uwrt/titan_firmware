#ifndef SAFETY_INTERFACE_H
#define SAFETY_INTERFACE_H

#include "profiler_defs.h"

#include "titan/safety.h"

// NOTE: If adding fault IDs make sure to update the fault_string_list as well

//      FAULT_WATCHDOG_RESET      0
//      FAULT_WATCHDOG_WARNING    1
#define FAULT_ROS_ERROR 2
#define FAULT_TIMER_MISSED 3
#define FAULT_ROS_BAD_COMMAND 4
#define FAULT_THRUSTER_TIMEOUT 5
#define FAULT_DEPTH_INIT_ERROR 6
#define FAULT_DEPTH_ERROR 7
#define FAULT_ADC_ERROR 8

static const char *const fault_string_list[] = { "FAULT_WATCHDOG_RESET",   "FAULT_WATCHDOG_WARNING",
                                                 "FAULT_ROS_ERROR",        "FAULT_TIMER_MISSED",
                                                 "FAULT_ROS_BAD_COMMAND",  "FAULT_THRUSTER_TIMEOUT",
                                                 "FAULT_DEPTH_INIT_ERROR", "FAULT_DEPTH_ERROR",
                                                 "FAULT_ADC_ERROR" };

extern bool safety_interface_kill_switch_refreshed;

#endif
