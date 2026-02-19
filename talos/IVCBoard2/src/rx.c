#include "rx.h"

#include "fft/fft.h"
#include "ivc.h"
#include "ros.h"

#include "pico/stdlib.h"
#include "titan/logger.h"

#include <math.h>
// make ring buffer of samples and have associated functions for pulling from it, getting a packet
signal_recv_t recv_data = { 0 };
frequency_bin_t fft_bins[] = { { "freq_low", FREQ_LOW_HZ - FFT_BIN_RANGE, FREQ_LOW_HZ + FFT_BIN_RANGE, 0 },
                               { "freq_high", FREQ_HIGH_HZ - FFT_BIN_RANGE, FREQ_HIGH_HZ + FFT_BIN_RANGE, 0 },
                               { "freq_sync", FREQ_SYNC_HZ - FFT_BIN_RANGE, FREQ_SYNC_HZ + FFT_BIN_RANGE, 0 } };
repeating_timer_t rx_timer = { 0 };
rx_capture_t rx = { 0 };
rx_control_t rx_ctrl = { 0 };
bool rx_packet_in_flight = false;

void rx_init() {
    // set the callback that fires when dma fills a swap buffer with data
    fft_setup(swap_buffer_handler);
    // start sampling on a swap buffer
    fft_sample(recv_data.swap_buffers[recv_data.buffer_select]);
    // recv_data.fft_target = -1;
}

bool rx_determine_valid_sample(float abs_conf_value) {
    return abs_conf_value >= 0.6f;
}

bool rx_cb(__unused repeating_timer_t *rt) {
    // LOG_INFO("IN RXCB");
    if (rx.buffer_full) {
        uint8_t got_packet = rx.mtu_data;
        // check crc
        ros_publish_rx(got_packet);
        rx.buffer_full = false;
        // handle packet
        // LOG_INFO("buffer full");
        rx_ctrl.receiving_packet = false;
        return false;
    }
    else {
        sample_t sample = rx_sample();
        if (sample != NONE && sample != SYNC) {
            rx.mtu_data = (sample & 0x01) << rx.write_pos;
            rx.buffer[rx.write_pos] = (uint8_t) sample;
            rx.write_pos++;
            if (rx.write_pos == 7) {  // fix this after
                rx.buffer_full = true;
            }
        }

        // handle bit: encode into mtu_data
        return true;
    }
}

void handle_incoming_packet() {
    if (!rx_ctrl.receiving_packet) {
        add_repeating_timer_ms(-SYMBOL_PERIOD_MS, rx_cb, NULL, &rx_timer);
        rx_ctrl.receiving_packet = true;
    }
    // making too many timers
    // recv_data.fft_target = -1;
}

void attempt_packet_read() {
    if (!rx_ctrl.receiving_packet) {
        add_repeating_timer_ms(-SYMBOL_PERIOD_MS, rx_cb, NULL, &rx_timer);
        rx_ctrl.receiving_packet = true;
    }
}

int8_t rx_handle_sample(float conf_value) {
    bool is_positive = conf_value >= 0.0f;
    float abs_conf_value = fabs(conf_value);
    int8_t value = -1;
    bool valid = rx_determine_valid_sample(abs_conf_value);
    if (valid) {
        if (is_positive) {
            value = 1;
            // ros_publish_rx_sample_debug(1);
        }
        else {
            value = 0;
            // ros_publish_rx_sample_debug(0);
        }
    }
    return value;
}

sample_t get_sample(float conf_value) {
    sample_t sample = NONE;

    bool is_positive = conf_value >= 0.0f;
    float abs = fabs(conf_value);
    bool is_valid = abs >= 0.6f;
    if (is_valid) {
        if (is_positive) {
            sample = HIGH;
        }
        else {
            sample = LOW;
        }
    }

    return sample;
}

// void rx_single_sample() {
//     fft_process(recv_data.swap_buffers[recv_data.fft_target], fft_bins, NUM_FFT_BINS);
//     float energy_at_low = fft_bins[FFT_LOW_IDX].amplitude;
//     float energy_at_high = fft_bins[FFT_HIGH_IDX].amplitude;
//     float energy_at_sync = fft_bins[FFT_SYNC_IDX].amplitude;
//     LOG_INFO("low: %f", energy_at_low);
//     LOG_INFO("high: %f", energy_at_high);
//     LOG_INFO("sync: %f", energy_at_sync);

//     bool sync_present = (energy_at_sync > (energy_at_low + energy_at_high));

//     float sum = energy_at_low + energy_at_high + 1e-6f;
//     float diff = energy_at_high - energy_at_low;
//     float conf = diff / sum;

//     sync_present ? (state = SYNC_FOUND) : rx_handle_sample(conf);
// }

bool is_idle(float e0, float e1, float e2) {
    return e0 < AMPLITUDE_IDLE_THRESHOLD && e1 < AMPLITUDE_IDLE_THRESHOLD && e2 < AMPLITUDE_IDLE_THRESHOLD;
}

sample_t rx_sample() {
    fft_process(recv_data.swap_buffers[recv_data.fft_target], fft_bins, NUM_FFT_BINS);
    float energy_at_low = fft_bins[FFT_LOW_IDX].amplitude;
    float energy_at_high = fft_bins[FFT_HIGH_IDX].amplitude;
    float energy_at_sync = fft_bins[FFT_SYNC_IDX].amplitude;
    // LOG_INFO("low: %f, high: %f, sync: %f", energy_at_low, energy_at_high, energy_at_sync);

    if (is_idle(energy_at_low, energy_at_high, energy_at_sync)) {
        return NONE;
    }

    if (energy_at_sync > (energy_at_low + energy_at_high)) {
        return SYNC;
    }

    float sum = energy_at_low + energy_at_high + 1e-6f;
    float diff = energy_at_high - energy_at_low;
    float conf = diff / sum;

    // int8_t sample = rx_handle_sample(conf);
    // if (sample == 1) {
    //     return HIGH;
    // }
    // else if (sample == 0) {
    //     return LOW;
    // }
    return get_sample(conf);
}

void rx_single_sample_int() {
    fft_process(recv_data.swap_buffers[recv_data.fft_target], fft_bins, NUM_FFT_BINS);
    float energy_at_low = fft_bins[FFT_LOW_IDX].amplitude;
    float energy_at_high = fft_bins[FFT_HIGH_IDX].amplitude;
    float energy_at_sync = fft_bins[FFT_SYNC_IDX].amplitude;

    bool sync_present = (energy_at_sync > (energy_at_low + energy_at_high));

    float sum = energy_at_low + energy_at_high + 1e-6f;
    float diff = energy_at_high - energy_at_low;
    int16_t conf = (diff / sum) * FLOAT_SCALE_MULTIPLIER;  // grab 3 decimal places as an integer

    sync_present ? (state = SYNC_FOUND) : rx_handle_sample(conf);
}

// callback to mark a swap buffer ready for processing, its full
void swap_buffer_handler() {
    recv_data.fft_target = recv_data.buffer_select;
    recv_data.buffer_select = !recv_data.buffer_select;
    fft_sample(recv_data.swap_buffers[recv_data.buffer_select]);
}
