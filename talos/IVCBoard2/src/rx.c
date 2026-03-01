#include "rx.h"

#include "fft/fft.h"
#include "ivc.h"
#include "ros.h"

#include "pico/stdlib.h"
#include "titan/logger.h"
#include "titan/queue.h"

#include <math.h>
#include <string.h>
// make ring buffer of samples and have associated functions for pulling from it, getting a packet
// signal_recv_t recv_data = { 0 };
frequency_bin_t fft_bins[] = { { "freq_low", FREQ_LOW_HZ - FFT_BIN_RANGE, FREQ_LOW_HZ + FFT_BIN_RANGE, 0 },
                               { "freq_high", FREQ_HIGH_HZ - FFT_BIN_RANGE, FREQ_HIGH_HZ + FFT_BIN_RANGE, 0 },
                               { "freq_sync", FREQ_SYNC_HZ - FFT_BIN_RANGE, FREQ_SYNC_HZ + FFT_BIN_RANGE, 0 } };
repeating_timer_t rx_timer = { 0 };
rx_control_t rx = { 0 };
rx_consensus_t consensus = { 0 };
// rx_control_t rx_ctrl = { 0 };
// struct QUEUE_DEFINE(uint8_t, 8) rx_msg_queue = { 0 };

bool rx_packet_in_flight = false;

void rx_init() {
    // set the callback that fires when dma fills a swap buffer with data
    fft_setup(swap_buffer_handler);
    // start sampling on a swap buffer
    // fft_sample(recv_data.swap_buffers[recv_data.buffer_select]);
    fft_sample(rx.recv.swap_buffers[rx.recv.buffer_select]);
    // recv_data.fft_target = -1;
}

bool rx_determine_valid_sample(float abs_conf_value) {
    return abs_conf_value >= 0.6f;
}

// void rx_enqueue_received_data(uint8_t data) {
//     if (QUEUE_FULL(&rx_msg_queue)) {
//         return;
//     }
//     uint8_t *entry = QUEUE_CUR_WRITE_ENTRY(&rx_msg_queue);
//     *entry = data;
//     // LOG_INFO("enqueueing %hhu\n", *entry);
//     QUEUE_MARK_WRITE_DONE(&rx_msg_queue);
//     // data_ready = true;
// }

// bool rx_dequeue_data(uint8_t *data) {
//     if (QUEUE_EMPTY(&rx_msg_queue)) {
//         return false;
//     }
//     uint8_t *read_data = QUEUE_CUR_READ_ENTRY(&rx_msg_queue);
//     *data = *read_data;
//     QUEUE_MARK_READ_DONE(&rx_msg_queue);
//     return true;
// }

bool rx_cb(__unused repeating_timer_t *rt) {
    // LOG_INFO("IN RXCB");
    // if (rx.buffer_full) {
    //     LOG_INFO("packet read complete with: %hhu", rx.mtu_data);
    //     rx.last_rx_value = rx.mtu_data;
    //     rx.publish_last_rx = true;
    //     // check crc
    //     rx.buffer_full = false;
    //     // handle packet
    //     // LOG_INFO("buffer full");
    //     rx_ctrl.receiving_packet = false;
    //     rx.mtu_data = 0;
    //     return false;
    // }
    // else {
    //     sample_t sample = rx_sample();
    //     if (sample != NONE && sample != SYNC) {
    //         rx.mtu_data |= (sample & 0x01) << rx.write_pos;
    //         rx.buffer[rx.write_pos] = (uint8_t) sample;
    //         rx.write_pos++;
    //         if (rx.write_pos == 7) {  // fix this after
    //             rx.buffer_full = true;
    //         }
    //         // LOG_INFO("building mtu data: [0x%X]\n", rx.mtu_data);
    //     }

    //     // handle bit: encode into mtu_data
    //     return true;
    // }
    if (rx.flags.buffer_full) {
        LOG_INFO("packet read complete with: %hhu", rx.mtu_data);
        rx.last_rx_value = rx.mtu_data;
        rx.flags.publish_last_rx = true;
        // check crc!
        rx.flags.buffer_full = false;
        rx.flags.receiving_packet = false;
        rx.mtu_data = 0;
        rx.write_pos = 0;
        return false;
    }
    else {
        sample_t sample = rx_sample();
        if (sample != NONE && sample != SYNC) {
            rx.mtu_data |= (sample & 0x01) << rx.write_pos;
            rx.buffer[rx.write_pos] = (uint8_t) sample;
            rx.write_pos++;
            if (rx.write_pos == 8) {
                rx.flags.buffer_full = true;
            }
            // LOG_INFO("building mtu data: [0x%X]\n", rx.mtu_data);
        }
        return true;
    }
}

void attempt_packet_read() {
    // if (!rx_ctrl.receiving_packet) {
    //     sleep_ms(0.5 * SYMBOL_PERIOD_MS);
    //     add_repeating_timer_ms(-SYMBOL_PERIOD_MS, rx_cb, NULL, &rx_timer);
    //     rx_ctrl.receiving_packet = true;
    // }
    // if (rx.publish_last_rx) {
    //     // ros_publish_rx(rx.last_rx_value);
    //     rx.publish_last_rx = false;
    // }
    if (!rx.flags.receiving_packet) {
        sleep_ms(0.5 * SYMBOL_PERIOD_MS);
        add_repeating_timer_ms(-SYMBOL_PERIOD_MS, rx_cb, NULL, &rx_timer);
        rx.flags.receiving_packet = true;
    }
    if (rx.flags.publish_last_rx) {
        // ros_publish_rx(rx.last_rx_value);
        rx.flags.publish_last_rx = false;
    }
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

bool is_idle(float e0, float e1, float e2) {
    return e0 < AMPLITUDE_IDLE_THRESHOLD && e1 < AMPLITUDE_IDLE_THRESHOLD && e2 < AMPLITUDE_IDLE_THRESHOLD;
}

sample_t rx_sample() {
    // fft_process(recv_data.swap_buffers[recv_data.fft_target], fft_bins, NUM_FFT_BINS);
    fft_process(rx.recv.swap_buffers[rx.recv.fft_target], fft_bins, NUM_FFT_BINS);
    float energy_at_low = fft_bins[FFT_LOW_IDX].amplitude;
    float energy_at_high = fft_bins[FFT_HIGH_IDX].amplitude;
    float energy_at_sync = fft_bins[FFT_SYNC_IDX].amplitude;
    // LOG_INFO("low: %f, high: %f, sync: %f", energy_at_low, energy_at_high, energy_at_sync);
    // if (energy_at_sync > 200.0f) {
    //     LOG_INFO("sync was: %04f", energy_at_sync);
    //     LOG_INFO("sync greater than threshold");
    // }

    if (is_idle(energy_at_low, energy_at_high, energy_at_sync)) {
        return NONE;
    }

    if (energy_at_sync > (energy_at_low + energy_at_high)) {
        // LOG_INFO("got sync pulse in sampling");
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

// callback to mark a swap buffer ready for processing, its full
void swap_buffer_handler() {
    // recv_data.fft_target = recv_data.buffer_select;
    // recv_data.buffer_select = !recv_data.buffer_select;
    // fft_sample(recv_data.swap_buffers[recv_data.buffer_select]);
    rx.recv.fft_target = rx.recv.buffer_select;
    rx.recv.buffer_select = !rx.recv.buffer_select;
    fft_sample(rx.recv.swap_buffers[rx.recv.buffer_select]);
    // sample_t sample = rx_observe();
    // LOG_INFO("pushing %hhu", sample);
    // consensus_push(sample);
}

// ============== with consensus =================

sample_t rx_observe() {
    // fft_process(recv_data.swap_buffers[recv_data.fft_target], fft_bins, NUM_FFT_BINS);
    fft_process(rx.recv.swap_buffers[rx.recv.fft_target], fft_bins, NUM_FFT_BINS);
    float energy_at_low = fft_bins[FFT_LOW_IDX].amplitude;
    float energy_at_high = fft_bins[FFT_HIGH_IDX].amplitude;
    float energy_at_sync = fft_bins[FFT_SYNC_IDX].amplitude;
    // LOG_INFO("low: %f, high: %f, sync: %f", energy_at_low, energy_at_high, energy_at_sync);
    // if (energy_at_sync > 200.0f) {
    //     LOG_INFO("sync was: %04f", energy_at_sync);
    //     LOG_INFO("sync greater than threshold");
    // }

    if (is_idle(energy_at_low, energy_at_high, energy_at_sync)) {
        return NONE;
    }

    if (energy_at_sync > (energy_at_low + energy_at_high)) {
        // LOG_INFO("got sync pulse in sampling");
        return SYNC;
    }

    float sum = energy_at_low + energy_at_high + 1e-6f;
    float diff = energy_at_high - energy_at_low;
    float conf = diff / sum;

    return get_sample(conf);
}

void consensus_push(sample_t observation) {
    // if (observation == NONE) {
    //     return;
    // }

    // LOG_INFO("pushing %hhu", observation);

    consensus.buffer[consensus.write_idx] = observation;
    consensus.write_idx = (consensus.write_idx + 1) % CONSENSUS_DEPTH;

    if (consensus.num_samples < CONSENSUS_DEPTH) {
        consensus.num_samples++;
    }
    // consensus.num_samples++;
}

void consensus_push_fast(sample_t observation) {
    if (observation == NONE) {
        return;
    }

    switch (observation) {
    case HIGH:
        consensus.votes.high++;
        break;
    case LOW:
        consensus.votes.low++;
        break;
    case SYNC:
        consensus.votes.sync++;
        break;
    }

    consensus.num_samples++;
}

bool consensus_stable_fast(sample_t *out_sample) {
    // LOG_INFO("CONSENSUS: \n\t sample count: %hhu \n\t votes: \n\t\thigh: %hhu, low: %hhu, sync: %hhu",
    //          consensus.num_samples, consensus.votes.high, consensus.votes.low, consensus.votes.sync);

    if (consensus.num_samples < CONSENSUS_DEPTH) {
        return false;
    }

    sample_t candidate = NONE;

    if (consensus.votes.sync >= MIN_CONSENSUS_VOTES) {
        candidate = SYNC;
    }
    else if (consensus.votes.high >= MIN_CONSENSUS_VOTES) {
        candidate = HIGH;
    }
    else if (consensus.votes.low >= MIN_CONSENSUS_VOTES) {
        candidate = LOW;
    }

    if (candidate == NONE) {
        return false;
    }

    *out_sample = candidate;
    return true;
}

bool consensus_stable(sample_t *out_sample) {
    if (consensus.num_samples < CONSENSUS_DEPTH) {
        return false;
    }

    uint32_t num_sync_votes = 0, num_high_votes = 0, num_low_votes = 0;

    for (uint32_t i = 0; i < CONSENSUS_DEPTH; i++) {
        switch (consensus.buffer[i]) {
        case HIGH:
            num_high_votes++;
            break;
        case LOW:
            num_low_votes++;
            break;
        case SYNC:
            num_sync_votes++;
            break;
        }
        // consensus.buffer[i] = NONE;
    }

    sample_t candidate = NONE;

    // if (num_sync_votes >= MIN_CONSENSUS_VOTES) {
    //     candidate = SYNC;
    // }
    // else if (num_high_votes >= MIN_CONSENSUS_VOTES) {
    //     candidate = HIGH;
    // }
    // else if (num_low_votes >= MIN_CONSENSUS_VOTES) {
    //     candidate = LOW;
    // }

    // if (candidate == NONE) {
    //     return false;
    // }

    if (num_sync_votes > num_high_votes && num_sync_votes > num_low_votes) {
        candidate = SYNC;
    }
    else if (num_high_votes > num_sync_votes && num_high_votes > num_low_votes) {
        candidate = HIGH;
    }
    else if (num_low_votes > num_sync_votes && num_low_votes > num_high_votes) {
        candidate = LOW;
    }

    // prevent repeated emissions
    // if (candidate == consensus.last_sample_seen) {
    //     return false;
    // }
    if (candidate != consensus.last_sample_seen) {
        rx.flags.on_new_symbol = true;
    }

    consensus.last_sample_seen = candidate;
    *out_sample = candidate;
    return true;
}

sample_t new_rx_sample() {
    consensus_push(rx_observe());
    sample_t resolved_sample;
    if (consensus_stable(&resolved_sample)) {
        return resolved_sample;
    }
    return NONE;
}

void new_attempt_packet_read() {
    if (!rx.flags.receiving_packet) {
        // sleep_ms(0.5 * SYMBOL_PERIOD_MS);
        add_repeating_timer_ms(-SYMBOL_PERIOD_MS, rx_cb, NULL, &rx_timer);
        rx.flags.receiving_packet = true;
    }
    if (rx.flags.publish_last_rx) {
        // ros_publish_rx(rx.last_rx_value);
        rx.flags.publish_last_rx = false;
    }
}

bool new_rx_cb(__unused repeating_timer_t *rt) {
    // LOG_INFO("IN RXCB");
    // if (rx.buffer_full) {
    //     LOG_INFO("packet read complete with: %hhu", rx.mtu_data);
    //     rx.last_rx_value = rx.mtu_data;
    //     rx.publish_last_rx = true;
    //     // check crc
    //     rx.buffer_full = false;
    //     // handle packet
    //     // LOG_INFO("buffer full");
    //     rx_ctrl.receiving_packet = false;
    //     rx.mtu_data = 0;
    //     return false;
    // }
    // else {
    //     sample_t sample = rx_sample();
    //     if (sample != NONE && sample != SYNC) {
    //         rx.mtu_data |= (sample & 0x01) << rx.write_pos;
    //         rx.buffer[rx.write_pos] = (uint8_t) sample;
    //         rx.write_pos++;
    //         if (rx.write_pos == 7) {  // fix this after
    //             rx.buffer_full = true;
    //         }
    //         // LOG_INFO("building mtu data: [0x%X]\n", rx.mtu_data);
    //     }

    //     // handle bit: encode into mtu_data
    //     return true;
    // }
    if (rx.flags.buffer_full) {
        LOG_INFO("packet read complete with: %hhu", rx.mtu_data);
        rx.last_rx_value = rx.mtu_data;
        rx.flags.publish_last_rx = true;
        // check crc!
        rx.flags.buffer_full = false;
        rx.flags.receiving_packet = false;
        rx.mtu_data = 0;
        rx.write_pos = 0;
        return false;
    }
    else {
        sample_t sample = new_rx_sample();

        if (sample == NONE || sample == SYNC) {
            return true;
        }
        rx.mtu_data |= (sample & 0x01) << rx.write_pos;
        rx.buffer[rx.write_pos] = (uint8_t) sample;
        rx.write_pos++;
        if (rx.write_pos == 8) {
            rx.flags.buffer_full = true;
        }
        // LOG_INFO("building mtu data: [0x%X]\n", rx.mtu_data);

        return true;
    }
}

// void resolve_symbol() {
//     sample_t sample;
//     if (consensus_stable(&sample)) {
//         if (sample == SYNC) {
//             rx.flags.receiving_packet = true;
//             rx.write_pos = 0;
//             rx.mtu_data = 0;
//         }
//         else if (rx.flags.receiving_packet) {
//             // push bit (write into mtu data)
//             rx.mtu_data |= (sample & 0x01) << rx.write_pos;
//             rx.buffer[rx.write_pos] = (uint8_t) sample;
//             rx.write_pos++;
//             if (rx.write_pos == 8) {
//                 rx.flags.buffer_full = true;
//                 rx.flags.receiving_packet = false;
//             }
//         }
//     }
// }

// ===================== take 2 ========================

// in tick, take a sample every time and put it in the consensus buffer.
// one consensus is made upon it having enough samples. that consensus will check sync
// if sync is present, set a timer (symbol period) to grab another consensus.
// at the end, publish last rx

void consensus_reset() {
    consensus.write_idx = 0;
    consensus.num_samples = 0;
    // memset(consensus.buffer, NONE, sizeof(consensus.buffer));
}

void consensus_reset_fast() {
    consensus.num_samples = 0;
    consensus.votes.high = consensus.votes.low = consensus.votes.sync = 0;
}

void rx_encode_sample(sample_t sample) {
    rx.mtu_data |= (sample & 0x01) << rx.write_pos;
    LOG_INFO("encoding into mtu data: 0x%X", rx.mtu_data);
    // rx.buffer[rx.write_pos] = (uint8_t) sample;
    rx.write_pos++;
    if (rx.write_pos == 8) {
        rx.flags.buffer_full = true;
        rx.flags.done_reading = true;
    }
}

void rx_reset() {
    rx.last_rx_value = rx.mtu_data;
    rx.mtu_data = 0;
    rx.write_pos = 0;
    rx.flags.publish_last_rx = true;
    rx.flags.done_reading = false;
}

bool cb(__unused repeating_timer_t *rt) {
    if (rx.flags.done_reading) {
        consensus_reset();
        rx_reset();
        rx.flags.receiving_packet = false;
        LOG_INFO("PACKET READ COMPLETE WITH: %hhu", rx.last_rx_value);
        return false;
    }
    else {
        sample_t sample;
        if (consensus_stable(&sample)) {
            // LOG_INFO("consensus stable in rx callback");
            LOG_INFO("sample in rx cb: %hhu", sample);
            if (sample == NONE || sample == SYNC) {
                consensus_reset();
                return false;
            }
            rx_encode_sample(sample);
            consensus_reset();
        }
        return true;
    }
}

void listen_for_packet() {
    if (!rx.flags.receiving_packet) {
        // consensus_wait();
        sleep_ms(0.9 * SYMBOL_PERIOD_MS);
        add_repeating_timer_ms(-SYMBOL_PERIOD_MS, cb, NULL, &rx_timer);
        rx.flags.receiving_packet = true;
    }
    if (rx.flags.publish_last_rx) {
        rx.flags.publish_last_rx = false;
        // ros_publish_rx(rx.last_rx_value);
    }
}

void listen_for_packet_no_timer() {
    if (!rx.flags.receiving_packet) {
        sample_t sample;
        if (consensus_stable(&sample)) {
            if (rx.flags.on_new_symbol) {
                LOG_INFO("on new symbol in listen: %hhu", sample);
                if (sample == NONE || sample == SYNC) {
                    consensus_reset();
                }
                rx_encode_sample(sample);
                consensus_reset();
                rx.flags.on_new_symbol = false;
            }
            else {
                rx_encode_sample(consensus.last_sample_seen);
                consensus_reset();
            }
            // LOG_INFO("consensus stable in rx callback");
        }
    }

    if (rx.flags.publish_last_rx) {
        rx.flags.publish_last_rx = false;
    }
}
