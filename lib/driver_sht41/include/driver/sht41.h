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

// PICO_CONFIG: SHT41_POLL_TIME_MS, time in between reading temperature and humidity, type=int, default=500, group=driver_sht41
#ifndef SHT41_POLL_TIME_MS
#define SHT41_POLL_TIME_MS 500
#endif

// maximum duration of time for a valid sht41 temperature and humidity data before it's considered invalid
#define SHT41_INVALID_TIMEOUT_MS (SHT41_POLL_TIME_MS + 100)

/**
 * @brief types of error sht41 driver could have
 *
 */
typedef enum {
    SHT41_ERROR_TIMER_SCHEDULE_FULL = 1,
    SHT41_ERROR_INITIAL_TIMER_SCHEDULE_FULL,
    SHT41_ERROR_CRC_INVALID,
    SHT41_ERROR_I2C_COMPLAINT
} sht41_error_code;

/**
 * @brief boolean if temperature and humidity data is ready to read
 *
 */
extern volatile bool sht41_temp_rh_set_on_read;

/**
 * @brief error callback.
 *
 */
typedef void (*sht41_error_cb)(const sht41_error_code error_type);

/**
 * @brief Initializes sht41 driver
 *
 * @param board_error_cb callback on error
 * @param i2c_bus_num the i2c bus number for sht41 driver
 */
void sht41_init(sht41_error_cb board_error_cb, uint8_t i2c_bus_num);

/**
 * @brief check the validity of data based on the last time it's retrieved
 *
 * @return true if lifetime of data hasn't exceeded SHT41_INVALID_TIMEOUT_MS
 * @return false otherwise
 */
bool sht41_is_valid(void);

/**
 * @brief read temperature
 *
 * @return float temperature in Celsius
 */
float sht41_read_temp(void);

/**
 * @brief read humidity
 *
 * @return float humidity in %RH
 */
float sht41_read_rh(void);
#endif
