#include "pico/time.h"
#include "driver/depth.h"
#include "ms5837.h"

#include "titan/logger.h"

#undef LOGGING_UNIT_NAME
#define LOGGING_UNIT_NAME "depth_sensor"


// Global variables, see header for docs
bool depth_initialized = false;
bool depth_set_on_read = false;

#define FLUID_DENSITY 997   // TODO: Make dynamic or at least fix for chlorinated/salt water

// ========================================
// Error Handling
// ========================================

/**
 * @brief The number of bad reads. Used as a counter before erroring on a comm error
 */
static int depth_num_bad_reads = 0;

static depth_error_cb depth_error_cb_func = NULL;
void depth_recoverable_err_cb(enum depth_error_event event) {
    if (event == DEPTH_ERROR_COMM_FAIL) {
        depth_num_bad_reads++;
        if (depth_num_bad_reads < DEPTH_BAD_READS_FAULT_COUNT) {
            return;
        }
    }

    if (depth_error_cb_func) {
        depth_error_cb_func(event, true);
    }
}

void depth_fatal_err_cb(enum depth_error_event event) {
    // Fatal errors will break the driver, so it becomes de-initialized.
    // Depth init should be re-ran to recover the sensor
    depth_initialized = false;

    if (depth_error_cb_func) {
        depth_error_cb_func(event, false);
    }
}


// ========================================
// Sensor Zeroing
// ========================================

/**
 * Magic value to report that the surface pressure is valid
 */
#define SURFACE_PRESSURE_VALID_MAGIC 0xAA556996

/**
 * @brief The surface pressure in tens of mbar
 * Generated during calibration
 */
static int32_t surface_pressure __attribute__((section(".uninitialized_data.depth_sensor")));
/**
 * @brief The surface pressure XOR SURFACE_PRESSURE_VALID_MAGIC to check that the value is valid on startup
 */
static int32_t surface_pressure_xor __attribute__((section(".uninitialized_data.depth_sensor")));
/**
 * @brief Contains SURFACE_PRESSURE_VALID_MAGIC to report that a valid surface pressure is contained in surface_pressure
*/
static int32_t surface_pressure_valid __attribute__((section(".uninitialized_data.depth_sensor")));

/**
 * @brief The number of times a zero reading has been added
 */
static int zero_count;

/**
 * @brief Callback after successfull ms5837 conversion during sensor zeroing
 *
 * @param pressure The pressure reading
 * @param temperature The temperature reading
 */
static void depth_zero_read_cb(int32_t pressure, int32_t temperature) {
    zero_count++;
    if (zero_count <= 40) {
        // First 20 readings aren't taken into account
        if (zero_count == 21) {
            surface_pressure = pressure;
        } else if (zero_count > 21) {
            surface_pressure = surface_pressure * .7 + pressure * .3;
        }

        ms5837_do_conversion(&depth_zero_read_cb, &depth_fatal_err_cb);
        return;
    }
    else {
        // Mark surface pressure as valid to allow recovery in the event of a watchdog reset
        surface_pressure_valid = SURFACE_PRESSURE_VALID_MAGIC;
        surface_pressure_xor = surface_pressure ^ SURFACE_PRESSURE_VALID_MAGIC;

        depth_initialized = true;
        if (add_alarm_in_ms(DEPTH_POLLING_RATE_MS, &depth_read_alarm_callback, NULL, true) < 0) {
            depth_fatal_err_cb(DEPTH_ERROR_ALARM_QUEUE_FULL);
        }
    }
}

/**
 * @brief Callback after successful ms5837 initialization.
 */
static void depth_init_complete(void) {
    if (surface_pressure_valid != SURFACE_PRESSURE_VALID_MAGIC ||
            (surface_pressure ^ SURFACE_PRESSURE_VALID_MAGIC) != surface_pressure_xor) {
        surface_pressure_valid = 0;
        zero_count = 0;
        ms5837_do_conversion(&depth_zero_read_cb, &depth_fatal_err_cb);
    } else {
        LOG_INFO("Depth surface pressure found... Skipping Zeroing of Depth");

        // Start depth sensor read task
        depth_initialized = true;
        if (add_alarm_in_ms(DEPTH_POLLING_RATE_MS, &depth_read_alarm_callback, NULL, true) < 0) {
            depth_fatal_err_cb(DEPTH_ERROR_ALARM_QUEUE_FULL);
        }
    }
}

// ========================================
// Sensor Reading
// ========================================

/**
 * @brief The last pressure reading from the sensor in tens of mbar
 */
static int32_t depth_pressure;
/**
 * @brief The last temperature reading from the sensor in hundreds of deg C
 */
static int32_t depth_temp;
/**
 * @brief The timeout of the last reading for when it will be invalid
 */
static absolute_time_t depth_current_read_timeout = {0};
/**
 * @brief Bool set to notify polling timer to stop and instead perform a calibration sequence.
 */
static bool depth_begin_recal = false;

/**
 * @brief Callback after successfull ms5837 conversion during sensor operation.
 *
 * @param pressure The pressure reading
 * @param temperature The temperature reading
 */
static void depth_read_cb(int32_t pressure, int32_t temperature) {
    depth_pressure = pressure;
    depth_temp = temperature;

    depth_current_read_timeout = make_timeout_time_ms(DEPTH_POLLING_RATE_MS * DEPTH_BAD_READS_FAULT_COUNT);
    depth_num_bad_reads = 0;
    depth_set_on_read = true;
}

/**
 * @brief Alarm callback to poll the depth sesnor during operation
 *
 * @param id The ID of the alarm that triggered the callback
 * @param user_data User provided data. This is NULL
 * @return int64_t If/How to restart the timer
 */
static int64_t depth_read_alarm_callback(__unused alarm_id_t id, __unused void *user_data) {
    if (ms5837_busy()) {
        depth_recoverable_err_cb(DEPTH_ERROR_OVERRUN);
    }
    else if (depth_begin_recal) {
        depth_begin_recal = false;
        depth_current_read_timeout = nil_time;
        surface_pressure_valid = 0;
        zero_count = 0;
        ms5837_do_conversion(&depth_zero_read_cb, &depth_fatal_err_cb);
        return 0;
    }
    else {
        ms5837_do_conversion(&depth_read_cb, &depth_recoverable_err_cb);
    }

    return DEPTH_POLLING_RATE_MS * 1000;
}

// ========================================
// Public Functions
// ========================================

void depth_init(unsigned int bus_num, enum depth_sensor_type sensor_type, depth_error_cb error_cb) {
    depth_error_cb_func = error_cb;
    ms5837_init(bus_num, sensor_type, depth_init_complete, depth_fatal_err_cb);
}

double depth_read(void) {
    return ((depth_pressure - surface_pressure)*100)/(FLUID_DENSITY*9.80665);
}

bool depth_reading_valid(void) {
    return depth_initialized && !time_reached(depth_current_read_timeout);
}

float depth_get_temperature(void) {
    return depth_temp / 100.0;
}

void depth_recalibrate(void) {
    depth_begin_recal = true;
}
