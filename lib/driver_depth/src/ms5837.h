#ifndef DRIVER__DEPTH_INTERNAL_H_
#define DRIVER__DEPTH_INTERNAL_H_

#include "driver/depth.h"

#include <stdint.h>

typedef void (*ms5837_init_cb)(void);
// Callback with pressure in Pascals and Temperature in hundredths of deg C
typedef void (*ms5837_read_cb)(int32_t pressure, int32_t temperature);
typedef void (*ms5837_error_cb)(enum depth_error_event event);

/**
 * @brief Initialize ms5837 pressure sensor.
 * @note This function can be called multiple times, so long as another operation is not pending
 *
 * @param bus_id The I2C bus ID for the sensor
 * @param sensor_type The sensor model to initialize
 * @param init_cb Callback upon successful init
 * @param error_cb Callback upon initialization error
 */
void ms5837_init(unsigned int bus_id, enum depth_sensor_type sensor_type, ms5837_init_cb init_cb,
                 ms5837_error_cb error_cb);

/**
 * @brief Executes a conversion
 * @note This function requires that the ms5837 successfully initialized
 *
 * @param read_cb Callback upon successful conversion read
 * @param error_cb Callback upon conversion error
 */
void ms5837_do_conversion(ms5837_read_cb read_cb, ms5837_error_cb error_cb);

/**
 * @brief Checks if an operation is currently being performed by the driver.
 * If the driver is busy, ms5837_init and ms5837_do_conversion cannot be called.
 *
 * @return true if driver is busy
 */
bool ms5837_busy(void);

#endif
