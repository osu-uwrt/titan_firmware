#ifndef _ACTUATOR_I2C__INTERFACE_H
#define _ACTUATOR_I2C__INTERFACE_H

#include "actuator_i2c/commands.h"
#include "actuator_i2c/crc8.h"
#include "actuator_i2c/responses.h"

#define ACTUATOR_I2C_ADDR 0x3A
#define ACTUATOR_EXPECTED_FIRMWARE_MAJOR ((int)1)
#define ACTUATOR_EXPECTED_FIRMWARE_MINOR ((int)0)

#endif