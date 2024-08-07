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
    DEF(FAULT_ROS_ERROR, 3)                                                                                            \
    DEF(FAULT_TIMER_MISSED, 4)                                                                                         \
    DEF(FAULT_ROS_BAD_COMMAND, 5)                                                                                      \
    DEF(FAULT_ADC_ERROR, 6)                                                                                            \
    DEF(FAULT_SHT41_ERROR, 7)                                                                                          \
    /* Raised if an actuator command could not successfully due to a hardware/firmware issue */                        \
    DEF(FAULT_ACTUATOR_FAILURE, 8)                                                                                     \
    /* Raised if the dynamixel reports that it has a hardware error */                                                 \
    DEF(FAULT_ACTUATOR_HW_FAULT, 9)                                                                                    \
    /* Raised if an actuator is unplugged, cleared when all actuators are plugged in */                                \
    DEF(FAULT_ACTUATOR_UNPLUGGED, 10)

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

// ==== Do not modify this code below this line ===
// This defines the enum of all faults using the XMACRO above
#define DEFINE_ENUMERATION(name, id) name = id,
enum safety_fault { XLIST_OF_FAULTS(DEFINE_ENUMERATION) };

#endif
