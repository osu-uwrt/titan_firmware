#include "dshot.h"

#include "hardware/pio.h"
#include "pico/stdlib.h"

#define FLAG_VALUE 123

int16_t target_rpm[NUM_THRUSTERS];

void core1_main() {
    multicore_fifo_push_blocking(FLAG_VALUE);

    uint32_t g = multicore_fifo_pop_blocking();

    int tick = 0;
    float min_frame_time = 0;

    if (g != FLAG_VALUE) {
        printf("Core 1 is not good!\n");
    }
    else {
        printf("Core 1 is good!");
    }

    int32_t throttle_commands[NUM_THRUSTERS] = { 0 };
    absolute_time_t last_time = nil_time;

    while (1) {
        absolute_time_t current_time = get_absolute_time();
        int64_t time_difference = 0;
        if (!is_nil_time(last_time)) {
            time_difference = absolute_time_diff_us(last_time, current_time);
        }

        // check_if_killed();
        dshot_send_commands(throttle_commands);

        absolute_time_t rpm_rx_timeout = make_timeout_time_us(RPM_RX_TIMEOUT_US);
        absolute_time_t min_frame_time = make_timeout_time_us(RPM_MIN_TIME_US);

        uint8_t pending = (1 << NUM_THRUSTERS) - 1;
        while (pending != 0 && !time_reached(rpm_rx_timeout)) {
            // Wait for an RPM message
            for (int i = 0; i < NUM_THRUSTERS; i++) {
                if (pio_sm_get_rx_fifo_level(DSHOT_PIO_BLOCK, i) > 0) {
                    // get RPM from ESC board
                    int16_t rpm = dshot_get_rpm();
                    // send RPM to controller
                    throttle_commands[i] = control(target_rpm[i], rpm, time_difference);
                    pending &= ~(1 << i);
                }
            }
        }
        if (!time_reached(min_frame_time)) {
            sleep_until(min_frame_time);
        }
        last_time = current_time;
    }
}
