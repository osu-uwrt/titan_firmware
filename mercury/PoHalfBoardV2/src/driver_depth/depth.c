#include "driver_depth/depth.h"

#include "ms5837.h"

#include "pico/time.h"
#include "titan/logger.h"

#undef LOGGING_UNIT_NAME
#define LOGGING_UNIT_NAME "depth_sensor"

/*
needs refactor to reflect only one depth sensor, or build in optional union to assign a pointer to in event
user has more chips
*/

// Global variables, see header for docs
volatile bool depth_initialized[NUM_DEPTH] = { false };  // REMOVE ARRAY
volatile bool depth_set_on_read[NUM_DEPTH] = { false };  // REMOVE ARRAY

#define FLUID_DENSITY 997  // TODO: Make dynamic or at least fix for chlorinated/salt water

static int64_t depth_read_alarm_callback(uint idx, __unused alarm_id_t id, __unused void *user_data);
static int64_t depth0_read_alarm_callback(__unused alarm_id_t id, __unused void *user_data);
static int64_t depth1_read_alarm_callback(__unused alarm_id_t id, __unused void *user_data);

// ========================================
// Error Handling
// ========================================

/**
 * @brief The number of bad reads. Used as a counter before erroring on a comm error
 */
static int depth_num_bad_reads[NUM_DEPTH] = { 0, 0 };

static depth_error_cb depth_error_cb_func[NUM_DEPTH] = { NULL, NULL };
void depth_recoverable_err_cb(uint idx, enum depth_error_event event) {
    if (event == DEPTH_ERROR_COMM_FAIL) {
        depth_num_bad_reads[idx] = depth_num_bad_reads[idx] + 1;
        if (depth_num_bad_reads[idx] < DEPTH_BAD_READS_FAULT_COUNT) {
            return;
        }
    }

    if (depth_error_cb_func[idx]) {
        depth_error_cb_func[idx](event, true);
    }
}

void depth0_recoverable_err_cb(enum depth_error_event event) {
    depth_recoverable_err_cb(0, event);
}

void depth1_recoverable_err_cb(enum depth_error_event event) {
    depth_recoverable_err_cb(1, event);
}

void depth_fatal_err_cb(uint idx, enum depth_error_event event) {
    // Fatal errors will break the driver, so it becomes de-initialized.
    // Depth init should be re-ran to recover the sensor
    depth_initialized[idx] = false;

    LOG_ERROR("Caught a depth fatal error");

    if (depth_error_cb_func[idx]) {
        depth_error_cb_func[idx](event, false);
    }
}

void depth0_fatal_err_cb(enum depth_error_event event) {
    depth_fatal_err_cb(0, event);
}

void depth1_fatal_err_cb(enum depth_error_event event) {
    depth_fatal_err_cb(1, event);
}

// ========================================
// Sensor Zeroing
// ========================================

/**
 * Magic value to report that the surface pressure is valid
 */
#define SURFACE_PRESSURE_VALID_MAGIC 0x55AA6996

/**
 * @brief The surface pressure in Pascals
 * Generated during calibration
 */
static int32_t surface_pressure[NUM_DEPTH] __attribute__((section(".uninitialized_data.depth_sensor")));
/**
 * @brief The surface pressure XOR SURFACE_PRESSURE_VALID_MAGIC to check that the value is valid on startup
 */
static int32_t surface_pressure_xor[NUM_DEPTH] __attribute__((section(".uninitialized_data.depth_sensor")));
/**
 * @brief Contains SURFACE_PRESSURE_VALID_MAGIC to report that a valid surface pressure is contained in surface_pressure
 */
static uint32_t surface_pressure_valid[NUM_DEPTH] __attribute__((section(".uninitialized_data.depth_sensor")));

/**
 * @brief The number of times a zero reading has been added
 */
static int zero_count[NUM_DEPTH];

static void depth0_zero_read_cb(int32_t pressure, __unused int32_t temperature);
static void depth1_zero_read_cb(int32_t pressure, __unused int32_t temperature);

/**
 * @brief Callback after successfull ms5837 conversion during sensor zeroing
 *
 * @param pressure The pressure reading
 * @param temperature The temperature reading
 */
static void depth_zero_read_cb(uint idx, int32_t pressure, __unused int32_t temperature) {
    LOG_INFO("Taking zero read");

    zero_count[idx] = zero_count[idx] + 1;
    if (zero_count[idx] <= 40) {
        // First 20 readings aren't taken into account
        if (zero_count[idx] == 21) {
            surface_pressure[idx] = pressure;
        }
        else if (zero_count[idx] > 21) {
            surface_pressure[idx] = surface_pressure[idx] * .7 + pressure * .3;
        }

        if (idx == 0)
            ms5837_do_conversion(0, &depth0_zero_read_cb, &depth0_fatal_err_cb);
        else
            ms5837_do_conversion(1, &depth1_zero_read_cb, &depth1_fatal_err_cb);

        return;
    }
    else {
        // Mark surface pressure as valid to allow recovery in the event of a watchdog reset
        surface_pressure_valid[idx] = SURFACE_PRESSURE_VALID_MAGIC;
        surface_pressure_xor[idx] = surface_pressure[idx] ^ SURFACE_PRESSURE_VALID_MAGIC;

        LOG_INFO("Marking depth_initialized as true with surface_pressure_valid: %u", surface_pressure_valid[idx]);

        depth_initialized[idx] = true;

        if (idx == 0) {
            if (add_alarm_in_ms(DEPTH_POLLING_RATE_MS, &depth0_read_alarm_callback, NULL, true) < 0) {
                depth0_fatal_err_cb(DEPTH_ERROR_ALARM_QUEUE_FULL);
            }
        }
        else {
            if (add_alarm_in_ms(DEPTH_POLLING_RATE_MS, &depth1_read_alarm_callback, NULL, true) < 0) {
                depth1_fatal_err_cb(DEPTH_ERROR_ALARM_QUEUE_FULL);
            }
        }
    }
}

static void depth0_zero_read_cb(int32_t pressure, __unused int32_t temperature) {
    depth_zero_read_cb(0, pressure, temperature);
}

static void depth1_zero_read_cb(int32_t pressure, __unused int32_t temperature) {
    depth_zero_read_cb(1, pressure, temperature);
}

/**
 * @brief Callback after successful ms5837 initialization.
 */
static void depth_init_complete(uint idx) {
    LOG_INFO("Depth init marked as complete");

    if (surface_pressure_valid[idx] != SURFACE_PRESSURE_VALID_MAGIC ||
        (surface_pressure[idx] ^ SURFACE_PRESSURE_VALID_MAGIC) != surface_pressure_xor[idx]) {
        surface_pressure_valid[idx] = 0;
        zero_count[idx] = 0;

        if (idx == 0)
            ms5837_do_conversion(idx, &depth0_zero_read_cb, &depth0_fatal_err_cb);
        else
            ms5837_do_conversion(idx, &depth1_zero_read_cb, &depth1_fatal_err_cb);
    }
    else {
        LOG_INFO("Depth surface pressure found... Skipping Zeroing of Depth");
        LOG_INFO("Marking depth_initialized as true");

        // Start depth sensor read task
        depth_initialized[idx] = true;
        if (idx == 0) {
            if (add_alarm_in_ms(DEPTH_POLLING_RATE_MS, &depth0_read_alarm_callback, NULL, true) < 0) {
                depth0_fatal_err_cb(DEPTH_ERROR_ALARM_QUEUE_FULL);
            }
        }
        else {
            if (add_alarm_in_ms(DEPTH_POLLING_RATE_MS, &depth1_read_alarm_callback, NULL, true) < 0) {
                depth1_fatal_err_cb(DEPTH_ERROR_ALARM_QUEUE_FULL);
            }
        }
    }
}

static void depth0_init_complete(void) {
    depth_init_complete(0);
}

static void depth1_init_complete(void) {
    depth_init_complete(1);
}

// ========================================
// Sensor Reading
// ========================================

/**
 * @brief The last pressure reading from the sensor in Pascals
 */
static int32_t depth_pressure[NUM_DEPTH];
/**
 * @brief The last temperature reading from the sensor in hundredths of deg C
 */
static int32_t depth_temp[NUM_DEPTH];
/**
 * @brief The timeout of the last reading for when it will be invalid
 */
static absolute_time_t depth_current_read_timeout[NUM_DEPTH] = { { 0 }, { 0 } };
/**
 * @brief Bool set to notify polling timer to stop and instead perform a calibration sequence.
 */
static bool depth_begin_recal[NUM_DEPTH] = { false, false };

/**
 * @brief Callback after successfull ms5837 conversion during sensor operation.
 *
 * @param pressure The pressure reading
 * @param temperature The temperature reading
 */
static void depth_read_cb(uint idx, int32_t pressure, int32_t temperature) {
    depth_pressure[idx] = pressure;
    depth_temp[idx] = temperature;

    depth_current_read_timeout[idx] = make_timeout_time_ms(DEPTH_POLLING_RATE_MS * DEPTH_BAD_READS_FAULT_COUNT);
    depth_num_bad_reads[idx] = 0;
    depth_set_on_read[idx] = true;
}

static void depth0_read_cb(int32_t pressure, int32_t temperature) {
    depth_read_cb(0, pressure, temperature);
}

static void depth1_read_cb(int32_t pressure, int32_t temperature) {
    depth_read_cb(1, pressure, temperature);
}

/**
 * @brief Alarm callback to poll the depth sesnor during operation
 *
 * @param id The ID of the alarm that triggered the callback
 * @param user_data User provided data. This is NULL
 * @return int64_t If/How to restart the timer
 */
static int64_t depth_read_alarm_callback(uint idx, __unused alarm_id_t id, __unused void *user_data) {
    if (ms5837_busy(idx)) {
        if (idx == 0)
            depth0_recoverable_err_cb(DEPTH_ERROR_OVERRUN);
        else
            depth1_recoverable_err_cb(DEPTH_ERROR_OVERRUN);
    }
    else if (depth_begin_recal[idx]) {
        depth_begin_recal[idx] = false;
        depth_current_read_timeout[idx] = nil_time;
        surface_pressure_valid[idx] = 0;
        zero_count[idx] = 0;

        if (idx == 0)
            ms5837_do_conversion(idx, &depth0_zero_read_cb, &depth0_fatal_err_cb);
        else
            ms5837_do_conversion(idx, &depth1_zero_read_cb, &depth1_fatal_err_cb);

        return 0;
    }
    else {
        if (idx == 0)
            ms5837_do_conversion(0, &depth0_read_cb, &depth0_recoverable_err_cb);
        else
            ms5837_do_conversion(1, &depth1_read_cb, &depth1_recoverable_err_cb);
    }

    return DEPTH_POLLING_RATE_MS * 1000;
}

static int64_t depth0_read_alarm_callback(__unused alarm_id_t id, __unused void *user_data) {
    return depth_read_alarm_callback(0, id, user_data);
}

static int64_t depth1_read_alarm_callback(__unused alarm_id_t id, __unused void *user_data) {
    return depth_read_alarm_callback(1, id, user_data);
}

// ========================================
// Public Functions
// ========================================

void depth_init(unsigned int bus_num, enum depth_sensor_type sensor_type, depth_error_cb error_cb) {
    depth_error_cb_func[bus_num] = error_cb;
    if (bus_num == 0)
        ms5837_init(bus_num, sensor_type, depth0_init_complete, depth0_fatal_err_cb);
    else
        ms5837_init(bus_num, sensor_type, depth1_init_complete, depth1_fatal_err_cb);

    LOG_INFO("surface_pressure_valid on startup: %u", surface_pressure_valid[bus_num]);
}

double depth_read(uint idx) {
    return (depth_pressure[idx] - surface_pressure[idx]) / (FLUID_DENSITY * 9.80665);
}

double pressure_read(int idx) {
    return depth_pressure[idx];
}

bool depth_reading_valid(uint idx) {
    // LOG_INFO("Depth init status is %d", depth_initialized[idx]);
    return depth_initialized[idx] && !time_reached(depth_current_read_timeout[idx]);
}

float depth_get_temperature(uint idx) {
    return depth_temp[idx] / 100.0;
}

void depth_recalibrate(uint idx) {
    depth_begin_recal[idx] = true;
}
