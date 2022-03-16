#ifndef _ACTUATOR_I2C__STATUS_H
#define _ACTUATOR_I2C__STATUS_H

#include <assert.h>
#include <stddef.h>
#include <stdint.h>

enum claw_state {
    CLAW_STATE_UNITIALIZED = 0,
    CLAW_STATE_UNKNOWN_POSITION = 1,
    CLAW_STATE_OPENED = 2,
    CLAW_STATE_CLOSED = 3,
    CLAW_STATE_OPENING = 4,
    CLAW_STATE_CLOSING = 5,
    CLAW_STATE_UNPLUGGED = 6,
    CLAW_STATE_MOTOR_DRIVER_FAULT = 7,
} __attribute__ ((packed));
static_assert(sizeof(enum claw_state) == 1, "Claw status enum did not pack properly");


enum torpedo_state {
    TORPEDO_STATE_UNITIALIZED = 0,
    TORPEDO_STATE_DISARMED = 1,
    TORPEDO_STATE_CHARGING = 2,
    TORPEDO_STATE_READY = 3,
    TORPEDO_STATE_FIRING = 4,
    TORPEDO_STATE_FIRED = 5,
} __attribute__ ((packed));
static_assert(sizeof(enum torpedo_state) == 1, "Torpedo status enum did not pack properly");


enum dropper_state {
    DROPPER_STATE_UNITIALIZED = 0,
    DROPPER_STATE_READY = 1,
    DROPPER_STATE_DROPPING = 2,
    DROPPER_STATE_DROPPED = 3,
} __attribute__ ((packed));
static_assert(sizeof(enum dropper_state) == 1, "Dropper status enum did not pack properly");


struct firmware_status {
    uint8_t fault_present:1;
    uint8_t firmware_version:7;
} __attribute__ ((packed)) status;
static_assert(sizeof(struct firmware_status) == 1, "Firmware status struct did not pack properly");


typedef struct actuator_i2c_status {
    uint8_t crc8;
    struct firmware_status firmware_status;
    enum claw_state claw_state;
    enum torpedo_state torpedo1_state;
    enum torpedo_state torpedo2_state;
    enum dropper_state dropper1_state;
    enum dropper_state dropper2_state;
} __attribute__ ((packed)) actuator_i2c_status_t;


#endif