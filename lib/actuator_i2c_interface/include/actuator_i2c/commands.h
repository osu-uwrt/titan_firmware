#ifndef _ACTUATOR_I2C__COMMANDS_H
#define _ACTUATOR_I2C__COMMANDS_H

#include <assert.h>
#include <stddef.h>
#include <stdint.h>

#define ACTUATOR_I2C_ADDR 0x3A

enum actuator_command {
    ACTUATOR_CMD_OPEN_CLAW = 1,
    ACTUATOR_CMD_CLAW_TIMING = 2,
    ACTUATOR_CMD_TEST = 5,
} __attribute__ ((packed));
static_assert(sizeof(enum actuator_command) == 1, "Actuator command enum did not pack properly");

#define ACTUATOR_CMD_OPEN_CLAW_LENGTH 0

struct test_cmd {
    uint32_t data1;
    uint32_t data2;
} __attribute__ ((packed));
#define ACTUATOR_CMD_TEST_LENGTH sizeof(struct test_cmd)

struct claw_timing_cmd {
    uint16_t open_time_ms;
    uint16_t close_time_ms;
} __attribute__ ((packed));
#define ACTUATOR_CMD_CLAW_TIMING_LENGTH sizeof(struct claw_timing_cmd)

typedef struct actuator_i2c_cmd {
    uint8_t crc8;
    enum actuator_command cmd_id;
    union {
        struct test_cmd test;
        struct claw_timing_cmd claw_timing;
    } data;
} __attribute__ ((packed)) actuator_i2c_cmd_t;

#endif