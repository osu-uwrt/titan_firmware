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

struct missing_timings_status {
    uint8_t claw_open_timing:1;
    uint8_t claw_close_timing:1;
    
    uint8_t marker_active_timing:1;

    uint8_t torpedo1_coil1_on_timing:1;
    uint8_t torpedo1_coil1_2_delay_timing:1;
    uint8_t torpedo1_coil2_on_timing:1;
    uint8_t torpedo1_coil2_3_delay_timing:1;
    uint8_t torpedo1_coil3_on_timing:1;

    uint8_t torpedo2_coil1_on_timing:1;
    uint8_t torpedo2_coil1_2_delay_timing:1;
    uint8_t torpedo2_coil2_on_timing:1;
    uint8_t torpedo2_coil2_3_delay_timing:1;
    uint8_t torpedo2_coil3_on_timing:1;

    uint8_t _unused:3;
};
static_assert(sizeof(struct missing_timings_status) == 2, "Missing timings status struct did not pack properly");

struct firmware_status {
    uint8_t version_major:4;
    uint8_t version_minor:4;
    uint8_t fault_list;
    struct missing_timings_status missing_timings;
} __attribute__ ((packed)) status;
static_assert(sizeof(struct firmware_status) == 4, "Firmware status struct did not pack properly");


struct actuator_i2c_status {
    struct firmware_status firmware_status;
    enum claw_state claw_state;
    enum torpedo_state torpedo1_state;
    enum torpedo_state torpedo2_state;
    enum dropper_state dropper1_state;
    enum dropper_state dropper2_state;
} __attribute__ ((packed));
#define ACTUATOR_STATUS_LENGTH sizeof(struct actuator_i2c_status)

enum actuator_command_result {
    ACTUATOR_RESULT_SUCCESSFUL = 0,
    ACTUATOR_RESULT_FAILED = 1,
    ACTUATOR_RESULT_RUNNING = 2,
}  __attribute__ ((packed));
static_assert(sizeof(enum claw_state) == 1, "Result enum did not pack properly");
#define ACTUATOR_RESULT_LENGTH sizeof(enum claw_state)

typedef struct actuator_i2c_response {
    uint8_t crc8;
    union {
        struct actuator_i2c_status status;
        enum actuator_command_result result;
    } data;
}  __attribute__ ((packed)) actuator_i2c_response_t;
#define ACTUATOR_BASE_RESPONSE_LENGTH offsetof(actuator_i2c_response_t, data)

#define ACTUATOR_STATUS_RESP_LENGTH (ACTUATOR_BASE_RESPONSE_LENGTH + ACTUATOR_STATUS_LENGTH)
#define ACTUATOR_RESULT_RESP_LENGTH (ACTUATOR_BASE_RESPONSE_LENGTH + ACTUATOR_RESULT_LENGTH)

#define ACTUATOR_GET_RESPONSE_SIZE(cmd_id) ( \
    cmd_id == ACTUATOR_CMD_GET_STATUS ? ACTUATOR_BASE_RESPONSE_LENGTH + ACTUATOR_STATUS_LENGTH : \
    cmd_id == ACTUATOR_CMD_OPEN_CLAW ? ACTUATOR_BASE_RESPONSE_LENGTH + ACTUATOR_RESULT_LENGTH : \
    cmd_id == ACTUATOR_CMD_CLOSE_CLAW ? ACTUATOR_BASE_RESPONSE_LENGTH + ACTUATOR_RESULT_LENGTH : \
    cmd_id == ACTUATOR_CMD_CLAW_TIMING ? ACTUATOR_BASE_RESPONSE_LENGTH + ACTUATOR_RESULT_LENGTH : \
    cmd_id == ACTUATOR_CMD_ARM_TORPEDO ? ACTUATOR_BASE_RESPONSE_LENGTH + ACTUATOR_RESULT_LENGTH : \
    cmd_id == ACTUATOR_CMD_DISARM_TORPEDO ? ACTUATOR_BASE_RESPONSE_LENGTH + ACTUATOR_RESULT_LENGTH : \
    cmd_id == ACTUATOR_CMD_FIRE_TORPEDO ? ACTUATOR_BASE_RESPONSE_LENGTH + ACTUATOR_RESULT_LENGTH : \
    cmd_id == ACTUATOR_CMD_TORPEDO_TIMING ? ACTUATOR_BASE_RESPONSE_LENGTH + ACTUATOR_RESULT_LENGTH : \
    cmd_id == ACTUATOR_CMD_DROP_MARKER ? ACTUATOR_BASE_RESPONSE_LENGTH + ACTUATOR_RESULT_LENGTH : \
    cmd_id == ACTUATOR_CMD_CLEAR_DROPPER_STATUS ? ACTUATOR_BASE_RESPONSE_LENGTH + ACTUATOR_RESULT_LENGTH : \
    cmd_id == ACTUATOR_CMD_MARKER_TIMING ? ACTUATOR_BASE_RESPONSE_LENGTH + ACTUATOR_RESULT_LENGTH : \
    cmd_id == ACTUATOR_CMD_KILL_SWITCH ? ACTUATOR_BASE_RESPONSE_LENGTH + ACTUATOR_RESULT_LENGTH : \
    cmd_id == ACTUATOR_CMD_RESET_ACTUATORS ? 0 : \
    0 \
)

#endif