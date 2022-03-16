#ifndef _ACTUATOR_I2C__CRC8_H
#define _ACTUATOR_I2C__CRC8_H

#include <stddef.h>
#include <stdint.h>
#include "actuator_i2c/commands.h"
#include "actuator_i2c/status.h"

uint8_t actuator_i2c_crc8_calc_raw(const uint8_t* data, size_t size);

static inline uint8_t actuator_i2c_crc8_calc_command(const actuator_i2c_cmd_t* cmd) {
    static_assert(offsetof(actuator_i2c_cmd_t, crc8) == 0, "crc must be first element in actuator_i2c_cmd");
    static_assert(sizeof(cmd->crc8) == 1, "crc must be 1 byte long in actuator_i2c_cmd");

    return actuator_i2c_crc8_calc_raw(((uint8_t*)cmd)+1, sizeof(*cmd)-1);
}

static inline uint8_t actuator_i2c_crc8_calc_status(const actuator_i2c_status_t* status) {
    static_assert(offsetof(actuator_i2c_status_t, crc8) == 0, "crc must be first element in actuator_i2c_status");
    static_assert(sizeof(status->crc8) == 1, "crc must be 1 byte long in actuator_i2c_status");

    return actuator_i2c_crc8_calc_raw(((uint8_t*)status)+1, sizeof(*status)-1);
}

#endif