#include "pico/binary_info.h"
#include "pico/stdlib.h"

#include "driver/canbus.h"
#include "driver/led.h"
#include "micro_ros_pico/transport_can.h"
#include "titan/logger.h"
#include "titan/version.h"
#include "titan/binary_info.h"

#include "dshot.h"
#include "ros.h"
#include "safety_interface.h"

#undef LOGGING_UNIT_NAME
#define LOGGING_UNIT_NAME "main"

#define UROS_CONNECT_PING_TIME_MS 1000
#define HEARTBEAT_TIME_MS 100
#define FIRMWARE_STATUS_TIME_MS 1000
#define LED_UPTIME_INTERVAL_MS 250
#define TELEM_PUBLISH_RATE_MS 500

// Initialize all to nil time
// For background timers, they will fire immediately
// For ros timers, they will be reset before being ticked by start_ros_timers
absolute_time_t next_heartbeat = {0};
absolute_time_t next_status_update = {0};
absolute_time_t next_telem_pub = {0};
absolute_time_t next_led_update = {0};
absolute_time_t next_connect_ping = {0};

uint8_t esc_board_num;
uint8_t client_id;
static void load_client_id(void){
    // Read board detection pin
    bi_decl_if_func_used(bi_1pin_with_name(BOARD_DET_PIN, "ESC Board Detection"))
    gpio_init(BOARD_DET_PIN);
    gpio_pull_down(BOARD_DET_PIN);
    sleep_us(5);    // Give time for pull up to settle

    // Report valid CAN IDs for this board
    bi_decl_if_func_used(bi_client_id(CAN_BUS_BOARD0_CLIENT_ID));
    bi_decl_if_func_used(bi_client_id(CAN_BUS_BOARD1_CLIENT_ID));

    // Determine ID
    esc_board_num = (gpio_get(BOARD_DET_PIN) ? 1 : 0);
    client_id = (esc_board_num ? CAN_BUS_BOARD1_CLIENT_ID : CAN_BUS_BOARD0_CLIENT_ID);
}

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
    next_telem_pub = make_timeout_time_ms(TELEM_PUBLISH_RATE_MS);
    dshot_command_received = false;
}

/**
 * @brief Ticks all ROS related code
 */
static void tick_ros_tasks() {
    if (timer_ready(&next_heartbeat, HEARTBEAT_TIME_MS, true)) {
        // RCSOFTRETVCHECK is used as important logs should occur within ros.c,
        RCSOFTRETVCHECK(ros_heartbeat_pulse(client_id));
    }

    if (timer_ready(&next_status_update, FIRMWARE_STATUS_TIME_MS, true)) {
        RCSOFTRETVCHECK(ros_update_firmware_status(client_id));
    }

    if (dshot_command_received) {
        dshot_command_received = false;
        RCSOFTRETVCHECK(ros_send_rpm(esc_board_num));
    }

    if (timer_ready(&next_telem_pub, TELEM_PUBLISH_RATE_MS, true)) {
        RCSOFTRETVCHECK(ros_send_telemetry(esc_board_num));
    }
}

static void tick_background_tasks() {
    canbus_tick();

    if (timer_ready(&next_led_update, LED_UPTIME_INTERVAL_MS, false)) {
        led_network_online_set(canbus_check_online());
    }
}


int main() {
    // Initialize stdio
    stdio_init_all();
    LOG_INFO("%s", FULL_BUILD_TAG);

    // Hook up safety
    safety_setup();
    load_client_id();
    led_init();
    micro_ros_init_error_handling();
    dshot_init();

    // Initialize ROS Transport
    if (!transport_can_init(client_id)) {
        // No point in continuing onwards from here, if we can't initialize CAN hardware might as well panic and retry
        panic("Failed to initialize CAN bus hardware!");
    }

    // Enter main loop
    bool ros_initialized = false;
    while(true) {
        // Do background tasks
        tick_background_tasks();

        // Handle ROS state logic
        if(is_ros_connected()) {
            if(!ros_initialized) {
                LOG_INFO("ROS connected");

                // Lower all ROS related faults as we've got a new ROS context
                safety_lower_fault(FAULT_ROS_ERROR);
                safety_lower_fault(FAULT_ROS_BAD_COMMAND);
                safety_lower_fault(FAULT_THRUSTER_TIMEOUT);

                if(ros_init(esc_board_num) == RCL_RET_OK) {
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
            LOG_INFO("Lost connection to ROS");
            ros_fini();
            safety_deinit();
            led_ros_connected_set(false);
            ros_initialized = false;
        } else {
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
