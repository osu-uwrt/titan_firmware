#include "nano_ros.h"
#include "nano_ros_codegen.h"
#include "protocol.h"
#include "safety_interface.h"
#include "transport_usb.h"

#include "pico/stdlib.h"
#include "titan/logger.h"
#include "titan/version.h"

#include <string.h>

#undef LOGGING_UNIT_NAME
#define LOGGING_UNIT_NAME "main"

absolute_time_t next_pub = { 0 };

transport_data_t tspt_data = { .err_code = 0, .timeout = 50 };
nros_transport_t transport = { .init = transport_usb_init,
                               .open = transport_usb_open,
                               .close = transport_usb_close,
                               .write = transport_usb_write,
                               .read = transport_usb_read,
                               .user_data = (void *) &tspt_data };

bool should_pub = false;
nros_frame_header_t header = { .topic_id = 5, .type = 0x02, .length = 0 };
riptide_msgs2__msg__FirmwareStatus_t fw = {
    .board_name = { 'j', 'e', 'f', 'f', '\0' },
    .bus_id = 12,
    .client_id = 8,
    .faults = UINT32_MAX,
    .kill_switches_asserting_kill = 74,
    .kill_switches_enabled = 74,
    .kill_switches_needs_update = 98,
    .kill_switches_timed_out = 33,
    .uptime_ms = 12345,
    .version_major = 6,
    .version_minor = 1,
    .version_release_type = 5,
};
uint8_t dropoff = 0;

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

static void tick_background_tasks() {
    // Update the LED (so it can alternate between colors if a fault is present)
    // This is only required if CAN transport is disabled, as the led_network_online_set will update the LEDs for us
    // if (timer_ready(&next_led_flash, LED_FLASH_INTERVAL_MS, false)) {
    //     led_state = !led_state;
    //     gpio_put(led_pin, led_state);
    // }

    if (timer_ready(&next_pub, 2000, false)) {
        should_pub = true;
    }

    // TODO: Put any code that should periodically occur here
    // Note that these tasks should NEVER block
    // Doing so makes calculating the worst case main loop time very difficult
    // If a task MUST block, do so in the main loop and make a comment for its worst case latency
    // If not, you will experience random watchdog timeouts which are very difficult to debug
}

void sub_cb(std_msgs__msg__UInt8_t *thing) {
    dropoff = thing->data;
    printf("got data: %hhu\n", dropoff);
}

void init_data() {
    nros_set_bit_sub_subscriber_cb(sub_cb);
}

int main() {
    // Initialize stdio
    stdio_init_all();
    // LOG_INFO("%s", FULL_BUILD_TAG);

    // Perform all initializations
    // NOTE: Safety must be the first thing up after stdio, so the watchdog will be enabled
    safety_setup();
    init_data();

    // TODO: Put any additional hardware initialization code here

    // Initialize safety before entering main loop
    // This makes the watchdog timer much more strict, as we aren't initializing hardware which might block for a while
    safety_init();
    nros_init(&transport, false);

    // Enter main loop
    while (true) {
        // Do background tasks
        tick_background_tasks();
        // TODO: Add your main task here
        // This is where your primary task which can block for a short period of time
        // (Note this MUST be gaurenteed to take less than your watchdog timeout defined in safety)
        // For example, you could add something which blocking reads and I2C sensor, and updates a screen
        // Just make sure your timeouts are designed properly to not time out safety

        if (should_pub) {
            should_pub = false;
            // fwrite(&header, sizeof(nros_frame_header_t), 1, stdout);
            // fflush(stdout);  // make sure it actually leaves the buffer now
            bool ok = nros_publish_firmware_status(&fw);
            // printf("published\n");
        }

        nros_spin();
        // Tick safety
        safety_tick();
    }

    return 0;
}
