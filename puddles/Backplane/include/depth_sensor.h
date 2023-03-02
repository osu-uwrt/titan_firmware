#ifndef _DEPTH_SENSOR_H
#define _DEPTH_SENSOR_H

#include <stdbool.h>
#include <stdint.h>

// PICO_CONFIG: PARAM_ASSERTIONS_ENABLED_DEPTH, Enable/disable assertions in the Depth Sensor module, type=bool, default=0, group=Depth
#ifndef PARAM_ASSERTIONS_ENABLED_DEPTH
#define PARAM_ASSERTIONS_ENABLED_DEPTH 0
#endif

// PICO_CONFIG: DEPTH_POLLING_RATE_MS, The depth sensor refresh rate in milliseconds, type=int, default=50, group=Depth
#ifndef DEPTH_POLLING_RATE_MS
#define DEPTH_POLLING_RATE_MS 50
#endif

// PICO_CONFIG: DEPTH_BAD_READS_FAULT_COUNT, The number of invalid reads from the depth sensor before a fault is raised, type=int, default=3, group=Depth
#ifndef DEPTH_BAD_READS_FAULT_COUNT
#define DEPTH_BAD_READS_FAULT_COUNT 3
#endif

/**
 * @brief Boolean for if depth is initialized.
 * This will be false until all calibration and zeroing is complete
 * This may never become true if the depth sensor fails to initialize
 */
extern bool depth_initialized;

/**
 * @brief Boolean which is set to true after each successful read.
 * This can be then set to false to poll for when a new reading is generated
 */
extern bool depth_set_on_read;

/**
 * @brief Reads the value from the depth sensor
 *
 * @attention depth_reading_valid must return true before calling this function
 *
 * @return double The raw depth reading
 */
double depth_read(void);

/**
 * @brief Returns the current temperature read from the depth sensor
 *
 * @attention depth_reading_valid must return true before calling this function
 *
 * @return float
 */
float depth_get_temperature(void);

/**
 * @brief Begins initialization of depth sensor
 *
 * @note The sensor is not initialized until depth_initialized is true
 */
void depth_init(void);

/**
 * @brief Returns if the depth reading is valid
 *
 * @return true If depth_read will return a valid reading
 * @return false If depth has not been initialized or the current depth reading is stale
 */
bool depth_reading_valid(void);

#endif