#include "ros.h"
#include "safety_interface.h"

#include "driver/async_i2c.h"
#include "driver/canbus.h"
#include "driver/depth.h"
#include "driver/led.h"
#include "driver/sht41.h"
#include "driver/status_strip.h"
#include "micro_ros_pico/transport_can.h"
#include "pico/binary_info.h"
#include "pico/stdlib.h"
#include "titan/binary_info.h"
#include "titan/logger.h"
#include "titan/version.h"

#undef LOGGING_UNIT_NAME
#define LOGGING_UNIT_NAME "main"

#define UROS_CONNECT_PING_TIME_MS 1000
#define HEARTBEAT_TIME_MS 100
#define FIRMWARE_STATUS_TIME_MS 1000
#define WATER_TEMP_PUBLISH_INTERVAL_MS 1000
#define LED_UPTIME_INTERVAL_MS 250

// Initialize all to nil time
// For background timers, they will fire immediately
// For ros timers, they will be reset before being ticked by start_ros_timers
absolute_time_t next_heartbeat = { 0 };
absolute_time_t next_status_update = { 0 };
absolute_time_t next_water_temp_publish = { 0 };

absolute_time_t next_led_update = { 0 };
absolute_time_t next_connect_ping = { 0 };

/**
 * @brief Check if a timer is ready. If so advance it to the next interval.
 *
 * This will also raise a fault if timers are missed
 *
 * @param next_fire_ptr A pointer to the absolute_time_t holding the time the timer should next fire
 * @param interval_ms The interval the timer fires at
 * @return true The timer has fired, any action which was waiting for this timer should occur
 * @return false The timer has not fired
 */
static inline bool timer_ready(absolute_time_t *next_fire_ptr, uint32_t interval_ms, bool error_on_miss) {
    absolute_time_t time_tmp = *next_fire_ptr;
    if (time_reached(time_tmp)) {
        time_tmp = delayed_by_ms(time_tmp, interval_ms);
        if (time_reached(time_tmp)) {
            unsigned int i = 0;
            while (time_reached(time_tmp)) {
                time_tmp = delayed_by_ms(time_tmp, interval_ms);
                i++;
            }
            LOG_WARN("Missed %u runs of %s timer 0x%p", i, (error_on_miss ? "critical" : "non-critical"),
                     next_fire_ptr);
            if (error_on_miss)
                safety_raise_fault_with_arg(FAULT_TIMER_MISSED, next_fire_ptr);
        }
        *next_fire_ptr = time_tmp;
        return true;
    }
    else {
        return false;
    }
}

/**
 * @brief Starts all ROS timers after ROS starts up
 *
 * Ensures that ROS timers will be staggered from ROS bringup and won't flood the queue all at once
 * This also prevents the timer code from warning about missing runs (as ROS hasn't been running)
 */
static void start_ros_timers() {
    next_heartbeat = make_timeout_time_ms(HEARTBEAT_TIME_MS);
    next_status_update = make_timeout_time_ms(FIRMWARE_STATUS_TIME_MS);
    next_water_temp_publish = make_timeout_time_ms(WATER_TEMP_PUBLISH_INTERVAL_MS);
}

/**
 * @brief Ticks all ROS related code
 */
static void tick_ros_tasks() {
    uint8_t client_id = CAN_BUS_CLIENT_ID;

    if (timer_ready(&next_heartbeat, HEARTBEAT_TIME_MS, true)) {
        // RCSOFTRETVCHECK is used as important logs should occur within ros.c,
        RCSOFTRETVCHECK(ros_heartbeat_pulse(client_id));
    }

    if (timer_ready(&next_status_update, FIRMWARE_STATUS_TIME_MS, true)) {
        RCSOFTRETVCHECK(ros_update_firmware_status(client_id));
    }

    // Send depth as soon as a new reading comes in
    if (depth_set_on_read) {
        depth_set_on_read = false;
        RCSOFTRETVCHECK(ros_update_depth_publisher());
    }

    if (depth_reading_valid() && timer_ready(&next_water_temp_publish, WATER_TEMP_PUBLISH_INTERVAL_MS, false)) {
        RCSOFTRETVCHECK(ros_update_water_temp_publisher());
    }

    if (sht41_temp_rh_set_on_read) {
        sht41_temp_rh_set_on_read = false;
        RCSOFTRETVCHECK(ros_update_temp_humidity_publisher());
    }
}

static void tick_background_tasks() {
    canbus_tick();

    if (timer_ready(&next_led_update, LED_UPTIME_INTERVAL_MS, false)) {
        led_network_online_set(canbus_check_online());
    }
}

static void depth_sensor_error_cb(enum depth_error_event event, bool recoverable) {
    if (recoverable) {
        safety_raise_fault_with_arg(FAULT_DEPTH_ERROR, event);
    }
    else {
        safety_raise_fault_with_arg(FAULT_DEPTH_INIT_ERROR, event);
    }
}

static void sht41_sensor_error_cb(const sht41_error_code error_type) {
    safety_raise_fault_with_arg(FAULT_SHT41_ERROR, error_type);
}

int main() {
    // Initialize stdio
    stdio_init_all();
    LOG_INFO("%s", FULL_BUILD_TAG);

    // Perform all initializations
    // NOTE: Safety must be the first thing up after stdio, so the watchdog will be enabled
    safety_setup();
    led_init();
    micro_ros_init_error_handling();

    // I2C Initialization
    bi_decl_if_func_used(bi_2pins_with_func(BOARD_SDA_PIN, BOARD_SCL_PIN, GPIO_FUNC_I2C));
    static_assert(BOARD_I2C == 0, "Board i2c expected on i2c0");
    async_i2c_init(BOARD_SDA_PIN, BOARD_SCL_PIN, -1, -1, 200000, 10);

    depth_init(BOARD_I2C, MS5837_02BA, &depth_sensor_error_cb);

    sht41_init(&sht41_sensor_error_cb, BOARD_I2C);

    // Status Strip Initialization
    bi_decl_if_func_used(bi_1pin_with_name(RGB_DATA_PIN, "Status RGB Strip"));
    status_strip_init(pio0, 0, RGB_DATA_PIN, true);
    status_strip_clear();

    // Initialize ROS Transports
    uint can_id = CAN_BUS_CLIENT_ID;
    bi_decl_if_func_used(bi_client_id(CAN_BUS_CLIENT_ID));
    if (!transport_can_init(can_id)) {
        // No point in continuing onwards from here, if we can't initialize CAN hardware might as well panic and retry
        panic("Failed to initialize CAN bus hardware!");
    }

    // Turn on fans
    // TODO: Move into fan controller module
    bi_decl_if_func_used(bi_1pin_with_name(FAN_SW_PIN, "Fan Control"));
    gpio_init(FAN_SW_PIN);
    gpio_put(FAN_SW_PIN, true);
    gpio_set_dir(FAN_SW_PIN, true);

    // Initialize Orin Switch to allow ROS control
    bi_decl_if_func_used(bi_1pin_with_name(ORIN_SW_PIN, "Orin Pwr Control"));
    gpio_init(ORIN_SW_PIN);
    gpio_put(ORIN_SW_PIN, true);
    gpio_set_dir(ORIN_SW_PIN, true);

    // Enter main loop
    bool ros_initialized = false;
    while (true) {
        // Do background tasks
        tick_background_tasks();

        // Handle ROS state logic
        if (is_ros_connected()) {
            if (!ros_initialized) {
                LOG_INFO("ROS connected");

                // Lower all ROS related faults as we've got a new ROS context
                safety_lower_fault(FAULT_ROS_ERROR);

                if (ros_init() == RCL_RET_OK) {
                    ros_initialized = true;
                    led_ros_connected_set(true);
                    safety_init();
                    start_ros_timers();
                }
                else {
                    LOG_ERROR("ROS failed to initialize.");
                    ros_fini();
                }
            }
            else {
                ros_spin_executor();
                tick_ros_tasks();
            }
        }
        else if (ros_initialized) {
            LOG_INFO("Lost connection to ROS");
            ros_fini();
            safety_deinit();
            led_ros_connected_set(false);
            status_strip_clear();  // Clear status strip on teardown of ROS
            ros_initialized = false;
        }
        else {
            if (time_reached(next_connect_ping)) {
                ros_ping();
                next_connect_ping = make_timeout_time_ms(UROS_CONNECT_PING_TIME_MS);
            }
        }

        // Tick safety
        safety_tick();
    }

    return 0;
}
