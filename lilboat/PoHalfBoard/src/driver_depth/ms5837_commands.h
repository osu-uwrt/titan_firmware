#ifndef _DEPTH_SENSOR_COMMANDS_H
#define _DEPTH_SENSOR_COMMANDS_H

#include "driver/async_i2c.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define DEPTH_I2C_ADDR 0x76

#define DEPTH_CMD_RESET 0x1E
#define DEPTH_CMD_CONVERT_D1(oversampling) (0x40 + (oversampling << 1))
#define DEPTH_CMD_CONVERT_D2(oversampling) (0x50 + (oversampling << 1))
#define DEPTH_CMD_ADC_READ 0x00
#define DEPTH_CMD_PROM_READ(addr) (0xA0 + (addr << 1))

#define DEPTH_PROM_EXPECTED_ID 0b0000001101000000
#define DEPTH_PROM_ID_MASK 0b0000111111100000

#define DEPTH_RESET_DELAY_MS 10
#define DEPTH_CONVERT_DELAY_US(oversample) (((5 * (1 << (8 + DEPTH_OVERSAMPLING))) / 2) + 2000)

#define DEPTH_OVERSAMPLING 5

#endif
