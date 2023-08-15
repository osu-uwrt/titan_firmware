#include "bq40z80.h"
#include "display.h"
#include "ros.h"
#include "safety_interface.h"

#include "driver/async_i2c.h"
#include "driver/canbus.h"
#include "driver/led.h"
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
#define PRESENCE_CHECK_INTERVAL_MS 1000
#define PRESENCE_TIMEOUT_COUNT 10
#define PWRCYCL_CHECK_INTERVAL_MS 250
#define PWR_CYCLE_DURATION_MS 10000
#define DISPLAY_UPDATE_INTERVAL_MS 1000

// Initialize all to nil time
// For background timers, they will fire immediately
// For ros timers, they will be reset before being ticked by start_ros_timers
absolute_time_t next_heartbeat = { 0 };
absolute_time_t next_status_update = { 0 };
absolute_time_t next_led_update = { 0 };
absolute_time_t next_connect_ping = { 0 };
absolute_time_t next_pack_present_update = { 0 };
absolute_time_t next_battery_status_update = { 0 };
absolute_time_t next_pwrcycl_update = { 0 };
absolute_time_t next_display_update = { 0 };

uint8_t presence_fail_count = 0;
bq_pack_info_t bq_pack_info;
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
    uint8_t client_id = can_id;

    if (timer_ready(&next_heartbeat, HEARTBEAT_TIME_MS, true)) {
        // RCSOFTRETVCHECK is used as important logs should occur within ros.c,
        RCSOFTRETVCHECK(ros_heartbeat_pulse(client_id));
    }

    // send the firmware status updates
    if (timer_ready(&next_status_update, FIRMWARE_STATUS_TIME_MS, true)) {
        RCSOFTRETVCHECK(ros_update_firmware_status(client_id));
    }

    // send the battery status updates
    if (timer_ready(&next_battery_status_update, BATTERY_STATUS_TIME_MS, true)) {
        RCSOFTRETVCHECK(ros_update_battery_status(bq_pack_info));
    }
}

static void tick_background_tasks() {
    canbus_tick();

    if (timer_ready(&next_led_update, LED_UPTIME_INTERVAL_MS, false)) {
        // update the RGB led
        led_network_online_set(canbus_check_online());

        // update the battery status leds
        bq_update_soc_leds();
    }

    // Update LCD if reed switch held and we are in time for a display update
    if (gpio_get(SWITCH_SIGNAL_PIN) && time_reached(next_display_update)) {
        next_display_update = make_timeout_time_ms(DISPLAY_UPDATE_INTERVAL_MS);

        // Show pack info
        display_show_stats(bq_pack_info.serial, bq_pack_soc(), bq_pack_voltage() / 1000.0);
    }

    if (timer_ready(&next_pack_present_update, PRESENCE_CHECK_INTERVAL_MS, false)) {
        // check if we need to update the presence counter
        if (!(canbus_check_online() || bq_pack_present())) {
            presence_fail_count++;
        }
        else {
            presence_fail_count = 0;
        }

        // if the presence counter times out, shut down
        if (presence_fail_count > PRESENCE_TIMEOUT_COUNT) {
            LOG_WARN("Pack not detected after %ds. Powering down!", PRESENCE_TIMEOUT_COUNT);
            gpio_put(PWR_CTRL_PIN, 0);
        }
    }

    display_check_poweroff();
}

int main() {
    // Latch RP2040 power to on
    gpio_init(PWR_CTRL_PIN);
    gpio_set_dir(PWR_CTRL_PIN, GPIO_OUT);
    gpio_put(PWR_CTRL_PIN, 1);

    gpio_init(SWITCH_SIGNAL_PIN);

    // Initialize stdio
    stdio_init_all();
    LOG_INFO("%s", FULL_BUILD_TAG);

    // Perform all initializations
    // NOTE: Safety must be the first thing up after stdio, so the watchdog will be enabled
    safety_setup();
    led_init();
    micro_ros_init_error_handling();
    async_i2c_init(PERIPH_SDA_PIN, PERIPH_SCL_PIN, -1, -1, 400000, 20);
    display_init();

    sleep_ms(1000);
    safety_tick();

    // start the bq40z80
    int err = bq_init();
    if (err > 0) {
        LOG_ERROR("Failed to initialize the bq40z80 after %d attempts", err);
        panic("BQ40Z80 Init failed!");
    }

    // grab the pack info from the bq40z80
    bq_pack_info = bq_pack_mfg_info();
    LOG_INFO("pack %s, mfg %d/%d/%d, SER# %d", bq_pack_info.name, bq_pack_info.mfg_mo, bq_pack_info.mfg_day,
             bq_pack_info.mfg_year, bq_pack_info.serial);

    // Initialize ROS Transports
    uint8_t sbh_mcu_serial_num = *(uint8_t *) (0x101FF000);
    if (sbh_mcu_serial_num == 0 || sbh_mcu_serial_num == 0xFF) {
        panic("Unprogrammed serial number");
    }

    if (bq_pack_info.serial != sbh_mcu_serial_num) {
        LOG_ERROR("BQ serial number (%d) does not match programmed serial (%d)", bq_pack_info.serial,
                  sbh_mcu_serial_num);
        safety_raise_fault(FAULT_BQ40_ERROR);
    }

    can_id = sbh_mcu_serial_num;
    if (!transport_can_init(can_id)) {
        // No point in continuing onwards from here, if we can't initialize CAN hardware might as well panic and retry
        panic("Failed to initialize CAN bus hardware!");
    }

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
        if (is_ros_connected()) {
            if (!ros_initialized) {
                LOG_INFO("ROS connected");
                display_show_ros_connect();

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
            display_show_ros_disconnect();
            ros_initialized = false;
        }
        else {
            if (time_reached(next_connect_ping)) {
                ros_ping();
                next_connect_ping = make_timeout_time_ms(UROS_CONNECT_PING_TIME_MS);
            }
        }

        // handle the power cycle outside of the timer environment as its a bit sensitive
        // this will block but also feed the watchdog. this will skew timing when called
        if (power_cycle_requested()) {
            bq_open_dschg_temp(PWR_CYCLE_DURATION_MS);
        }

        // Tick safety
        safety_tick();
    }
    return 0;
}
