#ifndef _SAFETY_H
#define _SAFETY_H

#include "safety/safety.h"

//      FAULT_WATCHDOG_RESET   0
#define FAULT_I2C_PROTO_ERROR  1
#define FAULT_I2C_ERROR        2
#define FAULT_CLAW_ERROR       3
// NOTE: There should only be up to fault id 7 since there is only an 8-bit byte for i2c fault status
static const char * const fault_string_list[] = {
    "FAULT_WATCHDOG_RESET",
    "FAULT_I2C_PROTO_ERROR",
    "FAULT_I2C_ERROR",
    "FAULT_CLAW_ERROR",
};


#define NUM_KILL_SWITCHES    1
#define KILL_SWITCH_I2C_MSG  0

#endif