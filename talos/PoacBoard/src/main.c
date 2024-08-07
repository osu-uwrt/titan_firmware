#include "actuators.h"
#include "ros.h"
#include "safety_interface.h"

#include "driver/async_i2c.h"
#include "driver/canbus.h"
#include "driver/led.h"
#include "driver/mcp3426.h"
#include "driver/sht41.h"
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
#define LED_UPTIME_INTERVAL_MS 250
#define KILLSWITCH_PUBLISH_TIME_MS 150
#define ELECTRICAL_READINGS_INTERVAL 1000
#define AUXSWITCH_INTERVAL 1000
#define ACTUATOR_STATUS_TIME_MS 500

// Initialize all to nil time
// For background timers, they will fire immediately
// For ros timers, they will be reset before being ticked by start_ros_timers
absolute_time_t next_heartbeat = { 0 };
absolute_time_t next_status_update = { 0 };
absolute_time_t next_led_update = { 0 };
absolute_time_t next_connect_ping = { 0 };
absolute_time_t next_killswitch_publish = { 0 };
absolute_time_t next_electrical_reading_publish = { 0 };
absolute_time_t next_auxswitch_publish = { 0 };
absolute_time_t next_actuator_status = { 0 };

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
static bool timer_ready(absolute_time_t *next_fire_ptr, uint32_t interval_ms, bool error_on_miss) {
    absolute_time_t time_tmp = *next_fire_ptr;
    if (time_reached(time_tmp)) {
        bool is_first_fire = is_nil_time(time_tmp);
        time_tmp = delayed_by_ms(time_tmp, interval_ms);
        if (time_reached(time_tmp)) {
            unsigned int i = 0;
            while (time_reached(time_tmp)) {
                time_tmp = delayed_by_ms(time_tmp, interval_ms);
                i++;
            }
            if (!is_first_fire) {
                LOG_WARN("Missed %u runs of %s timer 0x%p", i, (error_on_miss ? "critical" : "non-critical"),
                         next_fire_ptr);
                if (error_on_miss)
                    safety_raise_fault_with_arg(FAULT_TIMER_MISSED, next_fire_ptr);
            }
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
    next_killswitch_publish = make_timeout_time_ms(KILLSWITCH_PUBLISH_TIME_MS);
    next_electrical_reading_publish = make_timeout_time_ms(ELECTRICAL_READINGS_INTERVAL);
    next_auxswitch_publish = make_timeout_time_ms(AUXSWITCH_INTERVAL);
    next_actuator_status = make_timeout_time_ms(ACTUATOR_STATUS_TIME_MS);
}

/**
 * @brief Ticks all ROS related code
 */
static void tick_ros_tasks() {
    // Note that this is the *ONLY* function that any ros functions should be in, other than the executor
    // And in the executor, it should be the minimal amount of code to store the data received and get out

    // This is due to ROS timeouts/latency. In the event any reliable packet fails, it will block for the
    // whole 30 ms timeout. In order to ensure the 50ms minimum watchdog latency is met, there can only
    // be one timeout per loop. So, if any code errors for any reason, it should immediately return with an error
    // As this function should be  the only place with ROS code, it will exit before trying again, thus ensuring
    // that only 1 timeout occurs per safety tick.

    // If this is not followed, then the watchdog will reset if multiple timeouts occur within one tick

    uint8_t client_id = CAN_BUS_CLIENT_ID;

    if (timer_ready(&next_heartbeat, HEARTBEAT_TIME_MS, true)) {
        // RCSOFTRETVCHECK is used as important logs should occur within ros.c,
        RCSOFTRETVCHECK(ros_heartbeat_pulse(client_id));
    }

    if (timer_ready(&next_status_update, FIRMWARE_STATUS_TIME_MS, true)) {
        RCSOFTRETVCHECK(ros_update_firmware_status(client_id));
    }

    if (timer_ready(&next_killswitch_publish, KILLSWITCH_PUBLISH_TIME_MS, true) ||
        safety_interface_kill_switch_refreshed) {
        safety_interface_kill_switch_refreshed = false;
        RCSOFTRETVCHECK(ros_publish_killswitch());
    }

    if (timer_ready(&next_electrical_reading_publish, ELECTRICAL_READINGS_INTERVAL, true)) {
        RCSOFTRETVCHECK(ros_publish_electrical_readings());
    }

    if (timer_ready(&next_auxswitch_publish, AUXSWITCH_INTERVAL, true)) {
        RCSOFTRETVCHECK(ros_publish_auxswitch());
    }

    if (timer_ready(&next_actuator_status, ACTUATOR_STATUS_TIME_MS, true)) {
        RCSOFTRETVCHECK(ros_actuators_update_status());
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

static void mcp3426_error_callback(const struct async_i2c_request *req, uint32_t error_code) {
    // Mark as used in case debug logging is disabled
    (void) req;
    LOG_DEBUG("Error in mcp3426 driver request: 0x%p, error_code: 0x%08lx", req, error_code);
    safety_raise_fault_with_arg(FAULT_ADC_ERROR, error_code);
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

    // Turn on FAN to get cooling
    // TODO: Move to separate directory when tachometer added
    bi_decl_if_func_used(bi_1pin_with_name(FAN_SWITCH_PIN, "Fan Control"));
    gpio_init(FAN_SWITCH_PIN);
    gpio_put(FAN_SWITCH_PIN, true);
    gpio_set_dir(FAN_SWITCH_PIN, true);

    gpio_init(AUX_SWITCH_PIN);
    gpio_set_dir(AUX_SWITCH_PIN, false);

    gpio_init(STBD_STAT_PIN);
    gpio_init(PORT_STAT_PIN);
    gpio_set_dir(STBD_STAT_PIN, GPIO_IN);
    gpio_set_dir(PORT_STAT_PIN, GPIO_IN);
    gpio_disable_pulls(STBD_STAT_PIN);
    gpio_disable_pulls(PORT_STAT_PIN);

    actuators_initialize();

    // Initialize I2C
    bi_decl_if_func_used(bi_2pins_with_func(BOARD_SDA_PIN, BOARD_SCL_PIN, GPIO_FUNC_I2C));
    static_assert(BOARD_I2C == 0, "Board i2c expected on i2c0");
    async_i2c_init(BOARD_SDA_PIN, BOARD_SCL_PIN, -1, -1, 2000000, 10);
    mcp3426_init(BOARD_I2C, 0x68, mcp3426_error_callback);
    sht41_init(&sht41_sensor_error_cb, BOARD_I2C);

    // Initialize ROS Transports
    uint can_id = CAN_BUS_CLIENT_ID;
    bi_decl_if_func_used(bi_client_id(CAN_BUS_CLIENT_ID));
    if (!transport_can_init(can_id)) {
        // No point in continuing onwards from here, if we can't initialize CAN hardware might as well panic and retry
        panic("Failed to initialize CAN bus hardware!");
    }

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
                safety_lower_fault(FAULT_ROS_BAD_COMMAND);

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
