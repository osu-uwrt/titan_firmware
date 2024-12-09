#include "128D818.h"
#include "dshot.h"
#include "ros.h"
#include "safety_interface.h"
#include "time.h"

#include "driver/async_i2c.h"
#include "driver/depth.h"
#include "driver/led.h"
#include "driver/mcp3426.h"
#include "pico/binary_info.h"
#include "pico/stdlib.h"
#include "titan/logger.h"
#include "titan/version.h"

#ifdef MICRO_ROS_TRANSPORT_USB
#include "micro_ros_pico/transport_usb.h"
#endif

#ifdef MICRO_ROS_TRANSPORT_ETH
#include "driver/wiznet.h"
#include "micro_ros_pico/transport_eth.h"
#endif

#undef LOGGING_UNIT_NAME
#define LOGGING_UNIT_NAME "main"

#define UROS_CONNECT_PING_TIME_MS 1000
#define HEARTBEAT_TIME_MS 100
#define FIRMWARE_STATUS_TIME_MS 1000
#define ADC_TIME_MS 500
#define LED_UPTIME_INTERVAL_MS 250
#define KILLSWITCH_PUBLISH_TIME_MS 150
#define WATER_TEMP_PUBLISH_INTERVAL_MS 1000

// Initialize all to nil time
// For background timers, they will fire immediately
// For ros timers, they will be reset before being ticked by start_ros_timers
#define SUPRESS_BOOTUP_TIMER_MISS 1
absolute_time_t next_heartbeat = { 0 };
absolute_time_t next_status_update = { 0 };
absolute_time_t next_led_update = { 0 };
absolute_time_t next_connect_ping = { 0 };
absolute_time_t next_killswitch_publish = { 0 };
absolute_time_t next_water_temp_publish = { 0 };
absolute_time_t next_adc = { 0 };

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
    next_water_temp_publish = make_timeout_time_ms(WATER_TEMP_PUBLISH_INTERVAL_MS);
    next_adc = make_timeout_time_ms(ADC_TIME_MS);
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
    uint8_t client_id = 1;

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

    // Send depth as soon as a new reading comes in
    if (depth_set_on_read) {
        depth_set_on_read = false;
        RCSOFTRETVCHECK(ros_update_depth_publisher());
    }

    if (depth_reading_valid() && timer_ready(&next_water_temp_publish, WATER_TEMP_PUBLISH_INTERVAL_MS, false)) {
        RCSOFTRETVCHECK(ros_update_water_temp_publisher());
    }

    if (timer_ready(&next_adc, ADC_TIME_MS, false)) {
        RCSOFTRETVCHECK(adc_update());
    }

    // Send Dshot telemetry response after a dshot command is sent
    if (dshot_command_received) {
        dshot_command_received = false;
        RCSOFTRETVCHECK(ros_send_rpm());
    }
}

static void tick_background_tasks() {
    // Update the LED to report ethernet link status
    if (timer_ready(&next_led_update, LED_UPTIME_INTERVAL_MS, false)) {
        led_network_online_set(ethernet_check_online());
    }

    // Tick any ethernet tasks
    ethernet_tick();

    // TODO: Make not kill depth sensor
    // D818_read();
}

static void depth_sensor_error_cb(enum depth_error_event event, bool recoverable) {
    if (recoverable) {
        safety_raise_fault_with_arg(FAULT_DEPTH_ERROR, event);
    }
    else {
        safety_raise_fault_with_arg(FAULT_DEPTH_INIT_ERROR, event);
    }
}

static void mcp3426_error_callback(const struct async_i2c_request *req, uint32_t error_code) {
    // Mark as used in case debug logging is disabled
    (void) req;
    LOG_DEBUG("Error in mcp3426 driver request: 0x%p, error_code: 0x%08lx", req, error_code);

    safety_raise_fault_with_arg(FAULT_ADC_ERROR, error_code);
}

int main() {
    stdio_init_all();
    LOG_INFO("%s", FULL_BUILD_TAG);

    // Perform all initializations
    // NOTE: Safety must be the first thing up after stdio, so the watchdog will be enabled
    safety_setup();
    led_init();
    micro_ros_init_error_handling();

    static_assert(SENSOR_I2C == 0, "Sensor i2c expected on i2c0");
    static_assert(BOARD_I2C == 1, "Board i2c expected on i2c0");
    bi_decl_if_func_used(bi_2pins_with_func(SENSOR_SDA_PIN, SENSOR_SCL_PIN, GPIO_FUNC_I2C));
    bi_decl_if_func_used(bi_2pins_with_func(BOARD_SDA_PIN, BOARD_SCL_PIN, GPIO_FUNC_I2C));
    async_i2c_init(SENSOR_SDA_PIN, SENSOR_SCL_PIN, BOARD_SDA_PIN, BOARD_SCL_PIN, 400000, 10);
    depth_init(SENSOR_I2C, MS5837_30BA, &depth_sensor_error_cb);
    dshot_init();
    mcp3426_init(BOARD_I2C, 0x68, mcp3426_error_callback);
    D818_init();

    // init the CPU pwr ctrl system
    bi_decl_if_func_used(bi_1pin_with_name(PWR_CTL_CPU, "CPU Power Control"));
    gpio_init(PWR_CTL_CPU);
    gpio_set_dir(PWR_CTL_CPU, GPIO_OUT);
    gpio_put(PWR_CTL_CPU, 1);

    // init the acoustic pwr ctrl system off
    bi_decl_if_func_used(bi_1pin_with_name(PWR_CTL_ACC, "Acoustics Power Control"));
    gpio_init(PWR_CTL_ACC);
    gpio_set_dir(PWR_CTL_ACC, GPIO_OUT);
    gpio_put(PWR_CTL_ACC, 1);

    // init the aux switches as input
    bi_decl_if_func_used(bi_1pin_with_name(AUX_SW_SENSE, "Aux Switch"));
    gpio_init(AUX_SW_SENSE);
    gpio_set_dir(AUX_SW_SENSE, GPIO_IN);

    // Initialize ROS Transports
    if (!transport_eth_init()) {
        // No point in continuing onwards from here, if we can't initialize ETH hardware might as well panic and retry
        panic("Failed to initialize Ethernet hardware!");
    }

    // Enter main loop
    // This is split into two sections of timers
    // Those running with ROS, and those in the background
    // Note that both types of timers will need to conform to the minimal delay time, as there is around
    //   20ms of time worst case before the watchdog fires (as the ROS timeout is 30ms)
    // Meaning, don't block, either poll it in the background task or send it to an interrupt
    bool ros_initialized = false;
    while (true) {
        profiler_push(PROFILER_MAIN_LOOP);

        // Do background tasks
        profiler_push(PROFILER_BACKGROUND_TICK);
        tick_background_tasks();
        profiler_pop(PROFILER_BACKGROUND_TICK);

        // Handle ROS state logic
        if (is_ros_connected()) {
            if (!ros_initialized) {
                profiler_push(PROFILER_ROS_CONFIG);
                LOG_INFO("ROS connected");

                // Lower all ROS related faults as we've got a new ROS context
                safety_lower_fault(FAULT_ROS_ERROR);
                safety_lower_fault(FAULT_ROS_BAD_COMMAND);
                safety_lower_fault(FAULT_THRUSTER_TIMEOUT);

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
                profiler_pop(PROFILER_ROS_CONFIG);
            }
            else {
                profiler_push(PROFILER_EXECUTOR_TICK);
                ros_spin_executor();
                profiler_pop(PROFILER_EXECUTOR_TICK);

                profiler_push(PROFILER_ROS_TICK);
                tick_ros_tasks();
                profiler_pop(PROFILER_ROS_TICK);
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
            if (time_reached(next_connect_ping) && ethernet_check_online()) {
                ros_ping();
                next_connect_ping = make_timeout_time_ms(UROS_CONNECT_PING_TIME_MS);
            }
        }

        profiler_pop(PROFILER_MAIN_LOOP);

        // Tick safety
        safety_tick();
    }

    return 0;
}
