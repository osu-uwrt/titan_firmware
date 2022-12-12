#ifndef _SAFETY_H
#define _SAFETY_H

#include "safety/safety.h"

// NOTE: If adding fault IDs make sure to update the fault_string_list as well

//      FAULT_WATCHDOG_RESET   0
#define FAULT_ROS_SOFT_FAIL    1
#define FAULT_ROS_BAD_COMMAND  2
#define FAULT_DSHOT_ERROR      3
#define FAULT_THRUSTER_TIMEOUT 4
#define FAULT_ASYNC_I2C_ERROR  5
#define FAULT_DEPTH_INIT_ERROR 6
#define FAULT_DEPTH_ERROR      7
#define FAULT_ADC_ERROR        8
#define FAULT_BB_ADC_ERROR     9
#define FAULT_ESC_ADC_ERROR   10
#define FAULT_COOLING_STALE   11
#define FAULT_LOWBATT_STALE   12
#define FAULT_LOW_BATTERY     13
#define FAULT_ACTUATOR_FAIL   14
#define FAULT_NO_ACTUATOR     15
static const char * const fault_string_list[] = {
    "FAULT_WATCHDOG_RESET",
    "FAULT_ROS_SOFT_FAIL",
    "FAULT_ROS_BAD_COMMAND",
    "FAULT_DSHOT_ERROR",
    "FAULT_THRUSTER_TIMEOUT",
    "FAULT_ASYNC_I2C_ERROR",
    "FAULT_DEPTH_INIT_ERROR",
    "FAULT_DEPTH_ERROR",
    "FAULT_ADC_ERROR",
    "FAULT_BB_ADC_ERROR",
    "FAULT_ESC_ADC_ERROR",
    "FAULT_COOLING_STALE",
    "FAULT_LOWBATT_STALE",
    "FAULT_LOW_BATTERY",
    "FAULT_ACTUATOR_FAIL",
    "FAULT_NO_ACTUATOR",
};

// PICO_CONFIG: PARAM_ASSERTIONS_ENABLED_LIFETIME_CHECK, Enable/disable assertions for lifetime checking code, type=bool, default=0, group=Copro
#ifndef PARAM_ASSERTIONS_ENABLED_LIFETIME_CHECK
#define PARAM_ASSERTIONS_ENABLED_LIFETIME_CHECK 0
#endif

#endif