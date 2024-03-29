#ifndef SAFETY_INTERFACE_H
#define SAFETY_INTERFACE_H

#include "titan/safety.h"

// NOTE: If adding fault IDs make sure to update the fault_string_list as well

//      FAULT_WATCHDOG_RESET      0
//      FAULT_WATCHDOG_WARNING    1
#define FAULT_CAN_INTERNAL_ERROR  2
#define FAULT_CAN_RECV_ERROR      3

static const char * const fault_string_list[] = {
    "FAULT_WATCHDOG_RESET",
    "FAULT_WATCHDOG_WARNING",
    "FAULT_CAN_INTERNAL_ERROR",
    "FAULT_CAN_RECV_ERROR"
};


// If no kill switches defined, set NUM_KILL_SWITCHES = 0
enum kill_switch {
    NUM_KILL_SWITCHES = 0
};

#endif