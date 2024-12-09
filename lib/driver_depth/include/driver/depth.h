#ifndef DRIVER__DEPTH_SENSOR_H_
#define DRIVER__DEPTH_SENSOR_H_

#include <stdbool.h>
#include <stdint.h>

// PICO_CONFIG: PARAM_ASSERTIONS_ENABLED_DEPTH, Enable/disable assertions in the Depth Sensor module, type=bool, default=0, group=driver_depth
#ifndef PARAM_ASSERTIONS_ENABLED_DEPTH
#define PARAM_ASSERTIONS_ENABLED_DEPTH 0
#endif

// PICO_CONFIG: DEPTH_POLLING_RATE_MS, The depth sensor refresh rate in milliseconds, type=int, default=50, group=driver_depth
#ifndef DEPTH_POLLING_RATE_MS
#define DEPTH_POLLING_RATE_MS 50
#endif

// PICO_CONFIG: DEPTH_BAD_READS_FAULT_COUNT, The number of invalid reads from the depth sensor before a fault is raised, type=int, default=3, group=driver_depth
#ifndef DEPTH_BAD_READS_FAULT_COUNT
#define DEPTH_BAD_READS_FAULT_COUNT 5
#endif

enum depth_sensor_type { MS5837_02BA, MS5837_30BA };

enum depth_error_event {
    /**
     * @brief I2C error while sending reset command.
     * This is a non-recoverable error.
     */
    DEPTH_ERROR_RESET_CMD,
    /**
     * @brief The sensor PROM CRC did not match the computed calibration CRC.
     * This is a non-recoverable error.
     */
    DEPTH_ERROR_BAD_CRC,
    /**
     * @brief The periodic read timer fired before the previous read finished.
     * This is a recoverable error.
     */
    DEPTH_ERROR_OVERRUN,
    /**
     * @brief The Async I2C request queue was full when attempting to queue an initialization command.
     * This can either be a recoverable or non-recoverable error.
     */
    DEPTH_ERROR_I2C_QUEUE_FULL,
    /**
     * @brief All system alarms were in use when attempting to schedule a callback during a read.
     * This can either be a recoverable or non-recoverable error.
     */
    DEPTH_ERROR_ALARM_QUEUE_FULL,
    /**
     * @brief An error occurred while attempting to communicate with the depth sensor.
     * This can either be a recoverable or non-recoverable error.
     */
    DEPTH_ERROR_COMM_FAIL
};

typedef void (*depth_error_cb)(enum depth_error_event event, bool recoverable);

/**
 * @brief Boolean for if depth is initialized.
 * This will be false until all calibration and zeroing is complete
 * This may never become true if the depth sensor fails to initialize
 */
extern volatile bool depth_initialized;

/**
 * @brief Boolean which is set to true after each successful read.
 * This can be then set to false to poll for when a new reading is generated
 */
extern volatile bool depth_set_on_read;

/**
 * @brief Reads the value from the depth sensor.
 *
 * @attention depth_reading_valid must return true before calling this function
 *
 * @return double The raw depth reading
 */
double depth_read(void);

/**
 * @brief Returns the current temperature read from the depth sensor.
 *
 * @attention depth_reading_valid must return true before calling this function
 *
 * @return float The temperature in Deg C
 */
float depth_get_temperature(void);

/**
 * @brief Begins initialization of depth sensor.
 *
 * @param bus_num I2C bus index with depth sensor
 * @param sensor_type The depth sensor type used
 * @param error_cb Optional callback for error events. Can be NULL.
 *
 * @note The sensor is not initialized until depth_initialized is true
 */
void depth_init(unsigned int bus_num, enum depth_sensor_type sensor_type, depth_error_cb error_cb);

/**
 * @brief Returns if the depth reading is valid
 *
 * @return true If depth_read will return a valid reading
 * @return false If depth has not been initialized or the current depth reading is stale
 */
bool depth_reading_valid(void);

/**
 * @brief Begins a recalibration of the depth sensor.
 *
 * @note The recalibration will not begin until the next time a read is started (determined by DEPTH_POLLING_RATE_MS)
 */
void depth_recalibrate(void);

#endif
