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
    DEF(FAULT_CAN_INTERNAL_ERROR, 2)                                                                                   \
    DEF(FAULT_BQ25730_ERROR, 3)

// ===== Kill Switch Definitions =====
// If no kill switches defined, set NUM_KILL_SWITCHES = 0
enum kill_switch {

    // Used to automatically calculate number of kill switches
    // This must be the last enum
    NUM_KILL_SWITCHES = 0,
};

// ==== Do not modify this code below this line ===
// This defines the enum of all faults using the XMACRO above
#define DEFINE_ENUMERATION(name, id) name = id,
enum safety_fault { XLIST_OF_FAULTS(DEFINE_ENUMERATION) };

#endif
