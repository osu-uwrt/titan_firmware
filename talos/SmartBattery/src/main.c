#include "core1.h"
#include "display.h"
#include "ros.h"
#include "safety_interface.h"

#include "driver/async_i2c.h"
#include "driver/canbus.h"
#include "driver/led.h"
#include "driver/sht41.h"
#include "micro_ros_pico/transport_can.h"
#include "pico/stdlib.h"
#include "titan/logger.h"
#include "titan/version.h"

#undef LOGGING_UNIT_NAME
#define LOGGING_UNIT_NAME "main"

#define UROS_CONNECT_PING_TIME_MS 1000
#define HEARTBEAT_TIME_MS 100
#define FIRMWARE_STATUS_TIME_MS 1000
#define BATTERY_STATUS_TIME_MS 1000
#define LED_UPTIME_INTERVAL_MS 250
#define PWRCYCL_CHECK_INTERVAL_MS 250

// Initialize all to nil time
// For background timers, they will fire immediately
// For ros timers, they will be reset before being ticked by start_ros_timers
absolute_time_t next_heartbeat = { 0 };
absolute_time_t next_status_update = { 0 };
absolute_time_t next_led_update = { 0 };
absolute_time_t next_connect_ping = { 0 };
absolute_time_t next_battery_status_update = { 0 };

bq_mfg_info_t bq_mfg_info;
uint can_id;

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
    next_battery_status_update = make_timeout_time_ms(BATTERY_STATUS_TIME_MS);
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

    if (timer_ready(&next_heartbeat, HEARTBEAT_TIME_MS, true)) {
        // RCSOFTRETVCHECK is used as important logs should occur within ros.c,
        RCSOFTRETVCHECK(ros_heartbeat_pulse(can_id));
    }

    // send the firmware status updates
    if (timer_ready(&next_status_update, FIRMWARE_STATUS_TIME_MS, true)) {
        RCSOFTRETVCHECK(ros_update_firmware_status(can_id));
    }

    // send the battery status updates
    if (timer_ready(&next_battery_status_update, BATTERY_STATUS_TIME_MS, true)) {
        RCSOFTRETVCHECK(ros_update_battery_status(bq_mfg_info, can_id));
    }

    if (sht41_temp_rh_set_on_read) {
        sht41_temp_rh_set_on_read = false;
        RCSOFTRETVCHECK(ros_update_temp_humidity_publisher());
    }
}

static void tick_background_tasks() {
    canbus_tick();

    if (timer_ready(&next_led_update, LED_UPTIME_INTERVAL_MS, false)) {
        // update the RGB led
        led_network_online_set(canbus_check_online());
    }

    // Update from battery state
    batt_state_t batt_state = core1_get_batt_state();

    // Update LCD if reed switch held and we are in time for a display update
    display_tick(batt_state);

    // Handle CAN Bus initialization/deinitialization based on battery state
    bool can_bus_should_be_on = (batt_state == BATT_STATE_CHARGING || batt_state == BATT_STATE_DISCHARGING);
    static bool side_has_called = false;  // Ensures that the first call doesn't return true (if so, the data's stale)

    if (can_bus_should_be_on) {
        if (!canbus_initialized) {
            // Need to get the side detect pin status to see what side we're on
            bool side_detect_high;
            bool read_okay = core1_get_side_detect(&side_detect_high);
            if (read_okay && !side_has_called) {
                // Don't allow the first call to succeed, as the first call must always fail when queuing new cmds
                // If it returned true, then that means that there was stale data in there
                // This will ignore the first reading
                read_okay = false;
            }
            // We've called it once now, its okay to read it again
            side_has_called = true;

            // TODO: Negotiate CAN Bus ID conflicts if the fuse is blown

            // At this point we know that we should be connected to the battery
            // Fetch the manufacturing info so ROS will have it
            // If this fails, it'll try again next iteration (but it shouldn't since we were just told we're connected)
            if (read_okay) {
                read_okay = core1_get_pack_mfg_info(&bq_mfg_info);
            }

            if (read_okay) {
                // TODO: Validate this
                // We got the side, now set the CAN Bus ID
                can_id = (side_detect_high ? CAN_BUS_PORT_CLIENT_ID : CAN_BUS_STBD_CLIENT_ID);

                if (!transport_can_init(can_id)) {
                    safety_raise_fault(FAULT_CAN_INTERNAL_ERROR);
                }
                else {
                    led_network_enabled_set(true);
                    LOG_INFO("Enabled CAN Bus on Cable Connect");
                }
            }
        }
    }
    else {
        // Deinitialize CAN Bus if we don't have a valid cable attached
        if (canbus_initialized) {
            canbus_deinit();
            led_network_enabled_set(false);
            LOG_INFO("Disabled CAN Bus on Cable Removal");
        }

        // Make sure that side detect will refresh (and not run stale data)
        side_has_called = false;
    }

    // Handle auto-poweroff
    // We only want to keep firmware online if we are using CAN bus or if the display is on
    if (!can_bus_should_be_on && !display_check_on()) {
        if (gpio_get_out_level(PWR_CTRL_PIN)) {
            // Check out level to prevent log spam if the switch is held for whatever reason
            LOG_INFO("Both display is off and CAN Bus is offline - shutting off MCU to save power");
        }
        gpio_put(PWR_CTRL_PIN, 0);
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

    // Latch RP2040 power to on
    // Don't call gpio_init as this will disable our pad override we set in preinit before writing 1 to GPIO
    gpio_set_dir(PWR_CTRL_PIN, GPIO_OUT);
    gpio_put(PWR_CTRL_PIN, 1);
    gpio_set_function(PWR_CTRL_PIN, GPIO_FUNC_SIO);

    // Enable signal pin for reading reed switch
    gpio_init(SWITCH_SIGNAL_PIN);

    // Initialize all I2C Peripherals
    async_i2c_init(PERIPH_SDA_PIN, PERIPH_SCL_PIN, -1, -1, 400000, 20);
    display_init();
    sht41_init(&sht41_sensor_error_cb, PERIPH_I2C);

    // Load the serial number
    uint8_t sbh_mcu_serial_num = *(uint8_t *) (0x101FF000);
    if (sbh_mcu_serial_num == 0 || sbh_mcu_serial_num == 0xFF) {
        display_show_msg("No Ser# Err", "");
        panic("Unprogrammed serial number");
    }

    // Initialize the BQ40
    core1_init(sbh_mcu_serial_num);

    // Enter main loop
    // This is split into two sections of timers
    // Those running with ROS, and those in the background
    // Note that both types of timers will need to conform to the minimal delay time, as there is around
    //   20ms of time worst case before the watchdog fires (as the ROS timeout is 30ms)
    // Meaning, don't block, either poll it in the background task or send it to an interrupt
    bool ros_initialized = false;
    while (true) {
        // Do background tasks
        tick_background_tasks();

        // Handle ROS state logic
        // Also makes sure to handle if canbus gets deinitialized, ROS gets cleaned up properly
        if (canbus_initialized && is_ros_connected()) {
            if (!ros_initialized) {
                LOG_INFO("ROS connected");
                display_show_msg("ROS Connect", "");

                // Lower all ROS related faults as we've got a new ROS context
                safety_lower_fault(FAULT_ROS_ERROR);

                if (ros_init(sbh_mcu_serial_num) == RCL_RET_OK) {
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
            display_show_msg("ROS Lost", "");
            ros_initialized = false;
        }
        else {
            if (canbus_initialized && time_reached(next_connect_ping)) {
                ros_ping();
                next_connect_ping = make_timeout_time_ms(UROS_CONNECT_PING_TIME_MS);
            }
        }

        // Tick safety
        safety_tick();
    }
    return 0;
}
