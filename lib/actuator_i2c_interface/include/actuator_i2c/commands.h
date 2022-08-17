#ifndef _ACTUATOR_I2C__COMMANDS_H
#define _ACTUATOR_I2C__COMMANDS_H

#include <assert.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

enum actuator_command {
    ACTUATOR_CMD_GET_STATUS = 0,
    ACTUATOR_CMD_OPEN_CLAW = 1,
    ACTUATOR_CMD_CLOSE_CLAW = 2,
    ACTUATOR_CMD_CLAW_TIMING = 3,
    ACTUATOR_CMD_ARM_TORPEDO = 4,
    ACTUATOR_CMD_DISARM_TORPEDO = 5,
    ACTUATOR_CMD_FIRE_TORPEDO = 6,
    ACTUATOR_CMD_TORPEDO_TIMING = 7,
    ACTUATOR_CMD_DROP_MARKER = 8,
    ACTUATOR_CMD_CLEAR_DROPPER_STATUS = 9,
    ACTUATOR_CMD_DROPPER_TIMING = 10,
    ACTUATOR_CMD_KILL_SWITCH = 11,
    ACTUATOR_CMD_RESET_ACTUATORS = 12,
} __attribute__ ((packed));
static_assert(sizeof(enum actuator_command) == 1, "Actuator command enum did not pack properly");

#define ACTUATOR_CMD_GET_STATUS_LENGTH 0
#define ACTUATOR_CMD_OPEN_CLAW_LENGTH 0
#define ACTUATOR_CMD_CLOSE_CLAW_LENGTH 0
#define ACTUATOR_CMD_ARM_TORPEDO_LENGTH 0
#define ACTUATOR_CMD_DISARM_TORPEDO_LENGTH 0
#define ACTUATOR_CMD_CLEAR_DROPPER_STATUS_LENGTH 0
#define ACTUATOR_CMD_RESET_ACTUATORS_LENGTH 0

struct fire_torpedo_cmd {
    uint8_t torpedo_num;    // Note: Starts at 1
} __attribute__ ((packed));
#define ACTUATOR_CMD_FIRE_TORPEDO_LENGTH sizeof(struct fire_torpedo_cmd)


struct drop_marker_cmd {
    uint8_t dropper_num;    // Note: Starts at 1
} __attribute__ ((packed));
#define ACTUATOR_CMD_DROP_MARKER_LENGTH sizeof(struct drop_marker_cmd)

struct claw_timing_cmd {
    uint16_t open_time_ms;
    uint16_t close_time_ms;
} __attribute__ ((packed));
#define ACTUATOR_CMD_CLAW_TIMING_LENGTH sizeof(struct claw_timing_cmd)

enum torpedo_timing_type {
    ACTUATOR_TORPEDO_TIMING_COIL1_ON_TIME = 0,
    ACTUATOR_TORPEDO_TIMING_COIL1_2_DELAY_TIME = 1,
    ACTUATOR_TORPEDO_TIMING_COIL2_ON_TIME = 2,
    ACTUATOR_TORPEDO_TIMING_COIL2_3_DELAY_TIME = 3,
    ACTUATOR_TORPEDO_TIMING_COIL3_ON_TIME = 4,

    ACTUATOR_NUM_TORPEDO_TIMINGS
}  __attribute__ ((packed));
static_assert(sizeof(enum torpedo_timing_type) == 1, "Torpedo timing type enum did not pack properly");

// Coil number and coil active flag to timing index
// Note coil number starts at 0, and this will return the time this sequence will go for
// Meaning that COIL_TO_TIMING(0, true) will yield the timing the first coil must be on for (So timing index        `0)
#define COIL_ACTIVITY_TO_TIMING(coil, active) ((coil*2)+(active ? 0 : 1))


struct torpedo_timing_cmd {
    uint8_t torpedo_num;
    enum torpedo_timing_type timing_type;
    uint16_t time_us;
} __attribute__ ((packed));
#define ACTUATOR_CMD_TORPEDO_TIMING_LENGTH sizeof(struct torpedo_timing_cmd)

struct dropper_timing_cmd {
    uint16_t active_time_ms;
} __attribute__ ((packed));
#define ACTUATOR_CMD_DROPPER_TIMING_LENGTH sizeof(struct dropper_timing_cmd)

struct kill_switch_cmd {
    bool asserting_kill;
} __attribute__ ((packed));
#define ACTUATOR_CMD_KILL_SWITCH_LENGTH sizeof(struct kill_switch_cmd)

typedef struct actuator_i2c_cmd {
    uint8_t crc8;
    enum actuator_command cmd_id;
    union {
        struct fire_torpedo_cmd fire_torpedo;
        struct drop_marker_cmd drop_marker;
        struct claw_timing_cmd claw_timing;
        struct torpedo_timing_cmd torpedo_timing;
        struct dropper_timing_cmd dropper_timing;
        struct kill_switch_cmd kill_switch;
    } data;
} __attribute__ ((packed)) actuator_i2c_cmd_t;

#define ACTUATOR_BASE_CMD_LENGTH offsetof(actuator_i2c_cmd_t, data)

#define ACTUATOR_GET_CMD_SIZE(cmd_id) ( \
    cmd_id == ACTUATOR_CMD_GET_STATUS ? ACTUATOR_BASE_CMD_LENGTH + ACTUATOR_CMD_GET_STATUS_LENGTH : \
    cmd_id == ACTUATOR_CMD_OPEN_CLAW ? ACTUATOR_BASE_CMD_LENGTH + ACTUATOR_CMD_OPEN_CLAW_LENGTH : \
    cmd_id == ACTUATOR_CMD_CLOSE_CLAW ? ACTUATOR_BASE_CMD_LENGTH + ACTUATOR_CMD_CLOSE_CLAW_LENGTH : \
    cmd_id == ACTUATOR_CMD_CLAW_TIMING ? ACTUATOR_BASE_CMD_LENGTH + ACTUATOR_CMD_CLAW_TIMING_LENGTH : \
    cmd_id == ACTUATOR_CMD_ARM_TORPEDO ? ACTUATOR_BASE_CMD_LENGTH + ACTUATOR_CMD_ARM_TORPEDO_LENGTH : \
    cmd_id == ACTUATOR_CMD_DISARM_TORPEDO ? ACTUATOR_BASE_CMD_LENGTH + ACTUATOR_CMD_DISARM_TORPEDO_LENGTH : \
    cmd_id == ACTUATOR_CMD_FIRE_TORPEDO ? ACTUATOR_BASE_CMD_LENGTH + ACTUATOR_CMD_FIRE_TORPEDO_LENGTH : \
    cmd_id == ACTUATOR_CMD_TORPEDO_TIMING ? ACTUATOR_BASE_CMD_LENGTH + ACTUATOR_CMD_TORPEDO_TIMING_LENGTH : \
    cmd_id == ACTUATOR_CMD_DROP_MARKER ? ACTUATOR_BASE_CMD_LENGTH + ACTUATOR_CMD_DROP_MARKER_LENGTH : \
    cmd_id == ACTUATOR_CMD_CLEAR_DROPPER_STATUS ? ACTUATOR_BASE_CMD_LENGTH + ACTUATOR_CMD_CLEAR_DROPPER_STATUS_LENGTH : \
    cmd_id == ACTUATOR_CMD_DROPPER_TIMING ? ACTUATOR_BASE_CMD_LENGTH + ACTUATOR_CMD_DROPPER_TIMING_LENGTH : \
    cmd_id == ACTUATOR_CMD_KILL_SWITCH ? ACTUATOR_BASE_CMD_LENGTH + ACTUATOR_CMD_KILL_SWITCH_LENGTH : \
    cmd_id == ACTUATOR_CMD_RESET_ACTUATORS ? ACTUATOR_BASE_CMD_LENGTH + ACTUATOR_CMD_RESET_ACTUATORS_LENGTH : \
    0 \
)

#endif