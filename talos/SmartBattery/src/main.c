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
#define PRESENCE_CHECK_INTERVAL_MS 1000
#define PRESENCE_TIMEOUT_COUNT 10
#define PWRCYCL_CHECK_INTERVAL_MS 250

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

uint8_t presence_fail_count = 0;

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
        RCSOFTRETVCHECK(ros_update_battery_status(bq_mfg_info));
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

    // Update LCD if reed switch held and we are in time for a display update
    display_tick();

    if (timer_ready(&next_pack_present_update, PRESENCE_CHECK_INTERVAL_MS, false)) {
        // check if we need to update the presence counter
        if (!(canbus_check_online() || core1_check_present())) {
            presence_fail_count++;
        }
        else {
            presence_fail_count = 0;
        }

        // if the presence counter times out and the display is off, shut down
        if (presence_fail_count > PRESENCE_TIMEOUT_COUNT && !display_check_on()) {
            LOG_WARN("Pack not detected after %ds. Powering down!", PRESENCE_TIMEOUT_COUNT);
            gpio_put(PWR_CTRL_PIN, 0);
        }
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

    uint8_t retry = 0;
    do {
        if (++retry > 5) {
            display_show_msg("Init Failed", "");
            panic("BQ Init Failed");
        }
        if (retry > 1) {
            char msg[] = "BQ Con #X";
            msg[8] = retry + 48;
            char submsg[16];
            snprintf(submsg, sizeof(submsg), "Err: 0x%08lX", safety_fault_data[FAULT_BQ40_NOT_CONNECTED].extra_data);
            display_show_msg(msg, submsg);
        }
        sleep_ms(1000);
        safety_tick();
    } while (!core1_get_pack_mfg_info(&bq_mfg_info));

    LOG_INFO("pack %d, mfg %d/%d/%d, SER# %d", bq_mfg_info.device_type, bq_mfg_info.mfg_mo, bq_mfg_info.mfg_day,
             bq_mfg_info.mfg_year, bq_mfg_info.serial);

    // Initialize ROS Transports
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
