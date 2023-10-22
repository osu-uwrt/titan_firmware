#ifndef DRIVER__SHT41_H_
#define DRIVER__SHT41_H_

#include "driver/async_i2c.h"

#include <stdbool.h>
#include <stdint.h>
/**
 * @file driver/sht41.h
 *
 * @brief Driver to communicate with SHT41 temperature/humidity sensors.
 *
 */

typedef enum {
    SHT41_ERROR_TIMER_SCHEDULE_FULL = 1,
    SHT41_ERROR_CRC_INVALID,
    SHT41_ERROR_I2C_COMPLAINT
} sht41_error_code;

extern volatile bool sht41_temp_rh_set_on_read;

typedef void (*sht41_error_cb)(const sht41_error_code error_type, uint32_t i2c_error_code);

void sht41_init(sht41_error_cb board_error_cb);

bool sht41_is_valid(void);

int16_t sht41_read_temp(void);

int16_t sht41_read_rh(void);
#endif
