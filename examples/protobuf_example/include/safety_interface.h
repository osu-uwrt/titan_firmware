#ifndef SAFETY_INTERFACE_H
#define SAFETY_INTERFACE_H

#include "titan/safety.h"

// NOTE: If adding fault IDs make sure to update the fault_string_list as well

//      FAULT_WATCHDOG_RESET      0
//      FAULT_WATCHDOG_WARNING    1
#define FAULT_TIMER_MISSED 2
// TODO: Define any additional faults here

static const char *const fault_string_list[] = {
    "FAULT_WATCHDOG_RESET",
    "FAULT_WATCHDOG_WARNING",
    "FAULT_TIMER_MISSED",
};

// If no kill switches defined, set NUM_KILL_SWITCHES = 0
enum kill_switch {
    // EXAMPLE_KILL_SWITCH = 0,  // TODO: Define your kill switch here, or remove this line if no kill switches used
    // Used to automatically calculate number of kill switches
    // This must be the last enum
    NUM_KILL_SWITCHES
};

#endif
