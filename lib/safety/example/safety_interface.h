#ifndef SAFETY_INTERFACE_H
#define SAFETY_INTERFACE_H

#include "safety/safety.h"

// NOTE: If adding fault IDs make sure to update the fault_string_list as well

//      FAULT_WATCHDOG_RESET   0
#define FAULT_INVALID_SETUP    1
#define FAULT_HARDWARE_ERROR   2  // TODO: Replace these with implementation fault IDs

static const char * const fault_string_list[] = {
    "FAULT_WATCHDOG_RESET",
    "FAULT_INVALID_SETUP",
    "FAULT_HARDWARE_ERROR"
};


// TODO: Replace these with the kill switches for the implementation
// If no kill switches defined, set NUM_KILL_SWITCHES = 0
enum kill_switch {
    KILL_SWITCH_PHYSICAL = 0,
    KILL_SWITCH_I2C,

    // Used to automatically calculate number of kill switches
    // This must be the last enum
    NUM_KILL_SWITCHES
};

#endif