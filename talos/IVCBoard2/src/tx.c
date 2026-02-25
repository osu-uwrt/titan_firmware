#include "tx.h"

#include "fft/fft.h"
#include "ivc.h"

#include "hardware/adc.h"
#include "hardware/clocks.h"
#include "hardware/dma.h"
#include "hardware/pwm.h"
#include "pico/stdlib.h"
#include "titan/logger.h"
#include "titan/queue.h"

#include <stdbool.h>
#include <stdint.h>

#define SYSCLK_HZ clock_get_hz(clk_sys)
#define PWM_WRAP_VALUE ((int) (((float) SYSCLK_HZ) / 5000 / 255.0f) + 1.0f)

tx_control_t tx = { 0 };
repeating_timer_t tx_timer = { 0 };
struct QUEUE_DEFINE(uint8_t, 8) tx_msg_queue = { 0 };
// bool data_ready = false;

void tx_init() {
    // Find out which PWM slice is connected to TX_PIN (it's slice 0)
    gpio_set_function(TX_PIN, GPIO_FUNC_PWM);
    tx.pwm_slice_num = pwm_gpio_to_slice_num(TX_PIN);
    uint pwm_chan = pwm_gpio_to_channel(TX_PIN);

    pwm_set_wrap(tx.pwm_slice_num, PWM_WRAP_VALUE - 1);
    pwm_set_chan_level(tx.pwm_slice_num, pwm_chan, PWM_WRAP_VALUE / 2);

    // Select lpf output
    gpio_init(OUTPUT_SELECT_PIN);
    gpio_set_dir(OUTPUT_SELECT_PIN, GPIO_OUT);
    gpio_put(OUTPUT_SELECT_PIN, 1);
}

void tx_encode_bit(uint8_t bit) {
    float freq = (bit) ? FREQ_HIGH_HZ : FREQ_LOW_HZ;
    pwm_set_clkdiv(tx.pwm_slice_num, clock_get_hz(clk_sys) / (freq * PWM_WRAP_VALUE));
}

void tx_encode_sync() {
    pwm_set_clkdiv(tx.pwm_slice_num, clock_get_hz(clk_sys) / (FREQ_SYNC_HZ * PWM_WRAP_VALUE));
}

void tx_disable() {
    pwm_set_enabled(tx.pwm_slice_num, false);
    tx.flags.is_writing = false;
}

void tx_debug_khz(uint8_t khz) {
    float hz = khz * 1000.0f;
    pwm_set_enabled(tx.pwm_slice_num, true);
    pwm_set_clkdiv(tx.pwm_slice_num, clock_get_hz(clk_sys) / (hz * PWM_WRAP_VALUE));
    add_alarm_in_ms(50, tx_disable, NULL, true);
}

void tx_debug(uint8_t bit) {
    pwm_set_enabled(tx.pwm_slice_num, true);
    tx_encode_bit(bit);
    add_alarm_in_ms(50, tx_disable, NULL, true);
}

void tx_encode_packet(uint8_t packet) {
    // check state for sync_found. if found, get the hell out
}

bool tx_cb(__unused repeating_timer_t *rt) {
    // if (tx.done_writing) {
    //     LOG_INFO("done writing");
    //     pwm_set_enabled(tx.pwm_slice_num, false); tx_ctrl now tx
    //     // tx_ctrl.write_pos = 0;
    //     tx_ctrl.num_bits_written = 0;
    //     tx_ctrl.has_synced = false;
    //     tx_ctrl.is_writing = false;
    //     tx_ctrl.data_to_write = 0;
    //     tx_ctrl.done_writing = false;
    //     return false;
    // }
    // else {
    //     // pwm_set_enabled(tx_ctrl.pwm_slice_num, true);
    //     if (!tx_ctrl.has_synced) {
    //         LOG_INFO("sending sync pulse");
    //         tx_encode_sync();
    //         tx_ctrl.has_synced = true;
    //         return true;
    //     }
    //     // LOG_INFO("remaining data to write: 0x%X", tx_ctrl.data_to_write);
    //     tx_encode_bit(tx_ctrl.data_to_write & 0x01);
    //     tx_ctrl.data_to_write >>= 1;
    //     tx_ctrl.num_bits_written++;
    //     if (tx_ctrl.num_bits_written == 8) {
    //         tx_ctrl.done_writing = true;
    //     }
    //     return true;
    // }
    if (tx.flags.done_writing) {
        pwm_set_enabled(tx.pwm_slice_num, false);
        tx.num_bits_written = 0;
        tx.data_to_write = 0;
        tx.flags.has_synced = false;
        tx.flags.is_writing = false;
        tx.flags.done_writing = false;
    }
    else {
        if (!tx.flags.has_synced) {
            tx_encode_sync();
            tx.flags.has_synced = true;
            return true;
        }
        tx_encode_bit(tx.data_to_write & 0x01);
        tx.data_to_write >>= 1;
        tx.num_bits_written++;
        if (tx.num_bits_written == 8) {
            tx.flags.done_writing = true;
            // return false
        }
        return true;
    }
}

// void tx_encode_data() {
//     uint8_t data;
//     if (!tx_dequeue_data(&data)) {
//         return;
//     }
//     tx_ctrl.data_to_write = data;
//     if (!tx_ctrl.is_writing) {
//         pwm_set_enabled(tx_ctrl.pwm_slice_num, true);
//         add_repeating_timer_ms(-SYMBOL_PERIOD_MS, tx_cb, NULL, &tx_timer);
//         tx_ctrl.is_writing = true;
//     }
// }

void attempt_writing() {
    uint8_t data;
    if (!tx_dequeue_data(&data)) {
        return;
    }
    tx.data_to_write = data;
    if (!tx.flags.is_writing) {
        pwm_set_enabled(tx.pwm_slice_num, true);
        add_repeating_timer_ms(-SYMBOL_PERIOD_MS, tx_cb, NULL, &tx_timer);
        // tx_debug_khz(8);
        tx.flags.is_writing = true;
    }
}

void tx_enqueue_data(uint8_t data) {
    if (QUEUE_FULL(&tx_msg_queue)) {
        return;
    }
    uint8_t *entry = QUEUE_CUR_WRITE_ENTRY(&tx_msg_queue);
    *entry = data;
    LOG_INFO("enqueueing %hhu\n", *entry);
    QUEUE_MARK_WRITE_DONE(&tx_msg_queue);
    // data_ready = true;
}

// caller will pass a pointer where the data will be put
bool tx_dequeue_data(uint8_t *data) {
    if (QUEUE_EMPTY(&tx_msg_queue)) {
        // LOG_INFO("no data to pull");
        // data_ready = false;
        return false;
    }
    uint8_t *read_data = QUEUE_CUR_READ_ENTRY(&tx_msg_queue);
    *data = *read_data;
    QUEUE_MARK_READ_DONE(&tx_msg_queue);
    return true;
}

// void tick() {
//     pwm_set_enabled(tx_ctrl.pwm_slice_num, true);
//     uint8_t i = 2;
//     while (1) {
//         tx_encode_bit(0);
//         sleep_ms(1000);
//         tx_encode_bit(1);
//         sleep_ms(1000);
//         tx_encode_sync();
//         sleep_ms(1000);
//         i--;
//     }
// }
