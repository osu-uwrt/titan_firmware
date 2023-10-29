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
#define FAULT_ADC_ERROR 6
#define FAULT_SHT41_ERROR 7

static const char *const fault_string_list[] = {
    "FAULT_WATCHDOG_RESET", "FAULT_WATCHDOG_WARNING", "FAULT_CAN_INTERNAL_ERROR", "FAULT_ROS_ERROR",
    "FAULT_TIMER_MISSED",   "FAULT_ROS_BAD_COMMAND",  "FAULT_ADC_ERROR",          "FAULT_SHT41_ERROR",
};

/**
 * @brief The state of the physical kill switch (true if asserting kill)
 * Can be read to report if the physical kill switch state changes
 */
extern volatile bool safety_interface_physical_kill_asserting_kill;

/**
 * @brief Set every time the kill switch changes
 * Can be cleared by code monitoring for kill switch changes
 */
extern volatile bool safety_interface_kill_switch_refreshed;

#endif
