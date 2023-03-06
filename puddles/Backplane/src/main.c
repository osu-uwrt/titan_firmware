#include "pico/stdlib.h"

#include "async_i2c.h"
#include "basic_logger/logging.h"
#include "build_version.h"

#include "depth_sensor.h"
#include "mcp3426.h"
#include "dshot.h"
#include "ros.h"
#include "safety_interface.h"
#include "led.h"

#ifdef MICRO_ROS_TRANSPORT_USB
#include "micro_ros_pico/transport_usb.h"
#endif

#ifdef MICRO_ROS_TRANSPORT_ETH
#include "micro_ros_pico/transport_eth.h"
#include "eth_networking.h"
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
absolute_time_t next_heartbeat = {0};
absolute_time_t next_status_update = {0};
absolute_time_t next_led_update = {0};
absolute_time_t next_connect_ping = {0};
absolute_time_t next_killswitch_publish = {0};
absolute_time_t next_water_temp_publish = {0};
absolute_time_t next_adc = {0};

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
            unsigned int i = 0; \
            while(time_reached(time_tmp)) {
                time_tmp = delayed_by_ms(time_tmp, interval_ms);
                i++;
            }
            LOG_WARN("Missed %u runs of %s timer 0x%p", i, (error_on_miss ? "critical" : "non-critical"), next_fire_ptr);
            if (error_on_miss)
                safety_raise_fault(FAULT_TIMER_MISSED);
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

    if(timer_ready(&next_killswitch_publish, KILLSWITCH_PUBLISH_TIME_MS, true) || safety_interface_kill_switch_refreshed) {
        safety_interface_kill_switch_refreshed = false;
        RCSOFTRETVCHECK(ros_publish_killswitch());
    }

    // Send depth as soon as a new reading comes in
    if (depth_reading_valid() && depth_set_on_read) {
        depth_set_on_read = false;
        RCSOFTRETVCHECK(ros_update_depth_publisher());
    }

    if (depth_reading_valid() && timer_ready(&next_water_temp_publish, WATER_TEMP_PUBLISH_INTERVAL_MS, false)) {
        RCSOFTRETVCHECK(ros_update_water_temp_publisher());
    }

    if (timer_ready(&next_adc, ADC_TIME_MS, true)) {
        RCSOFTRETVCHECK(adc_update(client_id));
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


    // TODO: Put any code that should periodically occur here
    // read the aux switch state
    bool aux_state = gpio_get(AUX_SW_SENSE);

    mcp3426_read();
}


int main() {
    stdio_init_all();
    LOG_INFO("%s", FULL_BUILD_TAG);

    // Perform all initializations
    // NOTE: Safety must be the first thing up after stdio, so the watchdog will be enabled
    safety_setup();
    led_init();
    ros_rmw_init_error_handling();

    static_assert(SENSOR_I2C == 0, "Sensor i2c expected on i2c0");
    static_assert(BOARD_I2C == 1, "Board i2c expected on i2c0");
    async_i2c_init(SENSOR_SDA_PIN, SENSOR_SCL_PIN, BOARD_SDA_PIN, BOARD_SCL_PIN, 200000, 10);
    depth_init();
    dshot_init();
    mcp3426_init();

    // init the CPU pwr ctrl system
    gpio_init(PWR_CTL_CPU);
    gpio_set_dir(PWR_CTL_CPU, GPIO_OUT);
    gpio_put(PWR_CTL_CPU, 1);

    // init the acoustic pwr ctrl system off
    gpio_init(PWR_CTL_ACC);
    gpio_set_dir(PWR_CTL_ACC, GPIO_OUT);
    gpio_put(PWR_CTL_ACC, 0);

    // init the aux switches as input
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
    while(true) {
        // Do background tasks
        tick_background_tasks();

        // Handle ROS state logic
        if(is_ros_connected()) {
            if(!ros_initialized) {
                LOG_INFO("ROS connected");

                if(ros_init() == RCL_RET_OK) {
                    ros_initialized = true;
                    led_ros_connected_set(true);
                    safety_init();
                    start_ros_timers();
                } else {
                    LOG_ERROR("ROS failed to initialize.");
                    ros_fini();
                }
            } else {
                ros_spin_executor();
                tick_ros_tasks();
            }
        } else if(ros_initialized){
            LOG_INFO("Lost connection to ROS")
            ros_fini();
            safety_deinit();
            led_ros_connected_set(false);
            ros_initialized = false;
        } else {
            if (time_reached(next_connect_ping) && ethernet_check_online()) {
                ros_ping();
                next_connect_ping = make_timeout_time_ms(UROS_CONNECT_PING_TIME_MS);
            }
        }

        // Tick safety
        safety_tick();
    }

    return 0;
}