#ifndef SAFETY_INTERFACE_H
#define SAFETY_INTERFACE_H

#include "titan/safety.h"

// ===== Fault Definitions =====
// X macro to allow faults to be defined in one place
// Then all code which needs this fault array can use the XLIST_OF_FAULTS macro
// See https://en.wikipedia.org/wiki/X_macro for more info
// To add a new fault, add a new DEF(name, id) and add a backslash to the previous line

#define XLIST_OF_FAULTS(DEF)                                                                                           \
    XLIST_OF_LIBSAFETY_FAULTS(DEF) /* 0-1 reserved by safety */                                                        \
    DEF(FAULT_INVALID_SETUP, 2)    /* TODO Replace me - for example only */                                            \
    DEF(FAULT_HARDWARE_ERROR, 3)   /* TODO Replace me - for example only */
// TODO: Replace faults below with the implementation's fault IDs

// ===== Kill Switch Definitions =====
// TODO: Replace these with the kill switches for the implementation
// If no kill switches defined, set NUM_KILL_SWITCHES = 0
enum kill_switch {
    KILL_SWITCH_PHYSICAL = 0,
    KILL_SWITCH_I2C,

    // Used to automatically calculate number of kill switches
    // This must be the last enum
    NUM_KILL_SWITCHES
};

// ==== Do not modify this code below this line ===
// This defines the enum of all faults using the XMACRO above
#define DEFINE_ENUMERATION(name, id) name = id,
enum safety_fault { XLIST_OF_FAULTS(DEFINE_ENUMERATION) };

#endif
