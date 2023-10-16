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

enum sht41_error_code { SHT41_ERROR_TIMER_SCHEDULE_FULL = 1, SHT41_ERROR_CRC_INVALID, SHT41_ERROR_I2C_COMPLAINT };

typedef void (*sht41_error_cb)(const struct async_i2c_request *req, uint32_t error_code);  // FIXME needs inspection

void sht41_init(sht41_error_cb board_error_cb);

bool sht41_is_valid(void);

int16_t sht41_read_temp(void);

int16_t sht41_read_rh(void);
#endif
