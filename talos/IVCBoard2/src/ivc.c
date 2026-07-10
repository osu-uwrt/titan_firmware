#include "ivc.h"

#include "canmore.h"
#include "consensus.h"
#include "fft/fft.h"
#include "ros.h"
#include "rx.h"
#include "storage.h"
#include "tx.h"

#include "hardware/adc.h"
#include "hardware/clocks.h"
#include "hardware/dma.h"
#include "hardware/pwm.h"
#include "pico/stdlib.h"
#include "titan/logger.h"
#include "titan/queue.h"

#include <stdint.h>
#include <string.h>

#define TIMEOUT_COUNT 5

bool is_talos = false;
ivc_context_t context = { 0 };

bool started_listening_for_amplitude = false;
bool publish_amplitude = false;
repeating_timer_t amplitude_check_timer = { 0 };

static uint32_t none_observation_count = 0;

// testing flash
uint8_t test_write_buffer[NSAMP] = { 0xDE, 0xAD, 0xBE, 0xEF, 0xDE, 0xAD, 0xBE, 0xEF };
uint8_t test_read_buffer[8] = { 0 };
/**
 * @brief the interupt handler for swapping the process/fill buffers and resume fft sampling
 */
static void swap_buffer_handler() {
    context.rx.recv.fft_target = context.rx.recv.buffer_select;
    context.rx.recv.buffer_select = !context.rx.recv.buffer_select;
    fft_sample(context.rx.recv.swap_buffers[context.rx.recv.buffer_select]);
}

// if collecting adc samples to write to flash, call this in main
void data_ingest_init() {
    // storage_init(true);
    fft_setup(swap_buffer_handler, &context.rx.flags.is_pinger_mode, &context.rx.pinger_mode);
    fft_sample(context.rx.recv.swap_buffers[context.rx.recv.buffer_select]);

    gpio_init(BOARD_ID_PIN);
    gpio_set_dir(BOARD_ID_PIN, GPIO_IN);

    context.is_talos = !gpio_get(BOARD_ID_PIN);
    LOG_INFO("This board %s talos", context.is_talos ? "is" : "is not");
}

void ivc_init() {
    tx_init(&context);
    rx_init(&context, swap_buffer_handler);

#ifdef MICRO_ROS_TRANSPORT_CAN
    register_canmore_commands(&context);
#endif
    // #ifdef MICRO_ROS_TRANSPORT_USB
    //     context.rx.flags.is_pinger_mode = true;
    // #endif

    gpio_init(BOARD_ID_PIN);
    gpio_set_dir(BOARD_ID_PIN, GPIO_IN);

    context.is_talos = !gpio_get(BOARD_ID_PIN);
    LOG_INFO("This board %s talos", context.is_talos ? "is" : "is not");
    // set_topic_names(&ctx);
}

void debug_tx(uint8_t bit) {
    pwm_set_enabled(context.tx.pwm_slice_num, true);
    if (bit == SYNC) {
        tx_encode_sync(&context);
    }
    else {
        tx_encode_bit(&context, bit);
    }
    add_alarm_in_ms(500, tx_disable, (void *) &context, true);
}

void set_pinger_mode(int32_t khz) {
    if (khz == 0) {
        context.rx.flags.is_pinger_mode = false;
    }
    else if (!context.rx.flags.is_pinger_mode) {
        context.rx.flags.is_pinger_mode = true;
    }

    switch (khz) {
    case 20:
        context.rx.pinger_mode = FREQ_20KHZ;
        break;
    case 25:
        context.rx.pinger_mode = FREQ_25KHZ;
        break;
    case 30:
        context.rx.pinger_mode = FREQ_30KHZ;
        break;
    case 35:
        context.rx.pinger_mode = FREQ_35KHZ;
        break;
    case 37:
        context.rx.pinger_mode = FREQ_37KHZ;
        break;
    case 40:
        context.rx.pinger_mode = FREQ_40KHZ;
        break;
    default:
        break;
    }
}

void adc_sample_dump_tick() {
    static uint32_t pos = 0;
    static bool has_started = false;
    if (!has_started) {
        has_started = true;
        printf("START\n");
    }
    if (pos < (512 * 1024)) {  // im sorry
        uint8_t sample = storage_read_byte_at(pos++);
        printf("%d:%X\n", pos, sample);
    }
    else {
        ros_publish_adc_sample(1);
    }
}

void data_ingest_tick() {
    // for writing into flash
    static size_t num_buffers_written = 0;
    static bool has_flushed = false;
    static bool has_initialized = false;

    if (!has_initialized) {
        has_initialized = true;
        storage_init(false);
    }

    // if (num_buffers_written < 5240 && !has_flushed) {
    //     storage_write(context.rx.recv.swap_buffers[context.rx.recv.fft_target], NSAMP);
    //     // storage_write(test_write_buffer, NSAMP);
    //     num_buffers_written++;
    // }
    // else if (!has_flushed) {
    //     storage_flush();
    //     has_flushed = true;
    //     LOG_INFO("DONE STORING ADC");
    //     storage_read(test_read_buffer, 8);
    //     uint8_t *t = (uint8_t *) test_read_buffer;
    //     LOG_INFO("VALUE READ FROM START OF FLASH STORAGE: %02X %02X %02X %02X %02X %02X %02X %02X", t[0], t[1], t[2],
    //              t[3], t[4], t[5], t[6], t[7]);
    // }

    static bool has_read = false;
    if (!has_read) {
        storage_read(test_read_buffer, 8);
        uint8_t *t = test_read_buffer;
        LOG_INFO("VALUE READ FROM START OF FLASH STORAGE: %02X %02X %02X %02X %02X %02X %02X %02X", t[0], t[1], t[2],
                 t[3], t[4], t[5], t[6], t[7]);
        has_read = true;
    }
}

bool amplitude_cb() {
    process_fft(&context);
    float amp_low = get_low_amp();
    float amp_sync = get_sync_amp();
    float amp_high = get_high_amp();
    float ref1 = get_ref1_amp();
    float ref2 = get_ref2_amp();
    float ref3 = get_ref3_amp();
    float ref4 = get_ref4_amp();
    float pinger = get_pinger_amp();
    LOG_INFO("\nPINGER: %0.2f\nREF1: %0.2f\nREF2: %0.2f\nHIGH: %.2f\nLOW: %.2f\nSYNC: %.2f\nREF3: %0.2f\nREF4: %0.2f\n",
             pinger, ref1, ref2, amp_high, amp_low, amp_sync, ref3, ref4);
    // publish_amplitude = true;
    return true;
}

void amplitude_check_tick() {
    // process_fft(&context);
    if (!started_listening_for_amplitude) {
        started_listening_for_amplitude = true;
        add_repeating_timer_ms(100, amplitude_cb, NULL, &amplitude_check_timer);
    }
    if (publish_amplitude) {
        publish_amplitude = false;
        ros_publish_amplitude(get_high_amp());
    }
}

void ivc_tick() {
    // consensus_update(&context.consensus, rx_observe(&context));
    // if (rx_timeout_expired()) {
    //     rx_timeout(&context);
    // }

    // rx_timeout_start();
    // sample_t sample = rx_observe_3(&context);
    // if (sample == NONE) {
    //     rx_timeout_advance();
    // }
    // context.rx.flags.is_pinger_mode = true;

    if (context.rx.flags.is_pinger_mode) {
        //float avg;
        // if (rx_observe_pinger(&context, &avg)) {
        //     // LOG_INFO("PINGER AVG: %.2f", avg);
        //     ros_publish_pinger_amp(avg);
        // }
        rx_handle_pinger(&context);

        // LOG_INFO("is_pinger_mode: %s\ncurrent pinger mode: %d\n", context.rx.flags.is_pinger_mode ? "true" : "false",
        //          context.rx.pinger_mode);
    }
    else {
        consensus_update(&context.consensus, rx_observe_3(&context));
        // consensus_update(&context.consensus, sample);
        attempt_reading(&context);
        attempt_writing(&context);
    }
}

void nop_tick() {}
