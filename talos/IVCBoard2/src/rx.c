#include "rx.h"

#include "consensus.h"
#include "fft/fft.h"
#include "ivc.h"
#include "ros.h"

#include "pico/stdlib.h"
#include "titan/logger.h"
#include "titan/queue.h"

#include <math.h>
#include <string.h>

frequency_bin_t fft_bins[] = { { "freq_low", FREQ_LOW_HZ - FFT_BIN_RANGE, FREQ_LOW_HZ + FFT_BIN_RANGE, 0 },
                               { "freq_high", FREQ_HIGH_HZ - FFT_BIN_RANGE, FREQ_HIGH_HZ + FFT_BIN_RANGE, 0 },
                               { "freq_sync", FREQ_SYNC_HZ - FFT_BIN_RANGE, FREQ_SYNC_HZ + FFT_BIN_RANGE, 0 } };

void rx_init(ivc_context_t *ctx, void (*swap_buffer_handler)()) {
    // set the callback that fires when dma fills a swap buffer with data
    fft_setup(swap_buffer_handler);
    // start sampling on a swap buffer
    // fft_sample(recv_data.swap_buffers[recv_data.buffer_select]);
    fft_sample(ctx->rx.recv.swap_buffers[ctx->rx.recv.buffer_select]);
    // recv_data.fft_target = -1;
}

/**
 * @brief determines a samples value based on a confidence value -1.0 < conf_value < 1.0
 *
 * @param conf_value the confidence value
 * @return sample_t the interpretation of the confidence value
 */
static sample_t get_sample(float conf_value) {
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

/**
 * @brief determines if rx is idle based on a static threshold
 *
 * @param e0 amplitude 0
 * @param e1 amplitude 1
 * @param e2 amplitude 2
 * @return true if an amplitude is greater than AMPLITUDE_IDLE_THRESHOLD
 * @return false if all values fall below AMPLITUDUE_IDLE_THRESHOLD
 */
static bool is_idle(float e0, float e1, float e2) {
    return e0 < AMPLITUDE_IDLE_THRESHOLD && e1 < AMPLITUDE_IDLE_THRESHOLD && e2 < AMPLITUDE_IDLE_THRESHOLD;
}

sample_t rx_observe(ivc_context_t *ctx) {
    // fft_process(recv_data.swap_buffers[recv_data.fft_target], fft_bins, NUM_FFT_BINS);
    fft_process(ctx->rx.recv.swap_buffers[ctx->rx.recv.fft_target], fft_bins, NUM_FFT_BINS);
    float energy_at_low = fft_bins[FFT_LOW_IDX].amplitude;
    float energy_at_high = fft_bins[FFT_HIGH_IDX].amplitude;
    float energy_at_sync = fft_bins[FFT_SYNC_IDX].amplitude;
    // LOG_INFO("low: %f, high: %f, sync: %f", energy_at_low, energy_at_high, energy_at_sync);
    //      if (energy_at_sync > 200.0f) {
    //          LOG_INFO("sync was: %04f", energy_at_sync);
    //          LOG_INFO("sync greater than threshold");
    //      }

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

/**
 * @brief resets rx state after packet read completes
 *
 * @param ctx pointer to the main context struct
 */
static void rx_reset(ivc_context_t *ctx) {
    ctx->rx.packet_to_process = ctx->rx.current_packet;

    ctx->rx.last_rx_value = ctx->rx.mtu_data;
    ctx->rx.mtu_data = 0;
    ctx->rx.write_pos = 0;
    ctx->rx.flags.publish_last_rx = true;
    ctx->rx.flags.done_reading = false;
}

// TODO: test on hardware
// static void rx_encode_sample(ivc_context_t *ctx, sample_t sample) {
//     ctx->rx.current_packet |= (sample & 0x01) << ctx->rx.write_pos++;
//     LOG_INFO("encoding into mtu data: 0x%X", ctx->rx.mtu_data);
//     // rx.buffer[rx.write_pos] = (uint8_t) sample;
//     if (ctx->rx.write_pos == DATA_SIZE + CRC_SIZE) {
//         rx_reset(ctx);
//     }
// }

/**
 * @brief encodes a valid sample into a packet
 *
 * @param ctx pointer to the main context struct
 * @param sample the valid sample
 */
static void rx_encode_sample(ivc_context_t *ctx, sample_t sample) {
    ctx->rx.mtu_data |= (sample & 0x01) << ctx->rx.write_pos;
    LOG_INFO("encoding into mtu data: 0x%X", ctx->rx.mtu_data);
    // rx.buffer[rx.write_pos] = (uint8_t) sample;
    ctx->rx.write_pos++;
    if (ctx->rx.write_pos == DATA_SIZE) {
        rx_reset(ctx);
    }
}

/**
 * @brief determines how to handle a valid sample based on value and flags set, will set receiving packet flag to true
 *        if sample is a sync and we aren't currently receiving a packet
 *
 * @param ctx pointer to the main context struct
 * @param sample the valid sample
 */
static void handle_sample(ivc_context_t *ctx, sample_t sample) {
    if (sample == SYNC && !ctx->rx.flags.receiving_packet) {
        ctx->rx.flags.receiving_packet = true;
    }
    else if (ctx->rx.flags.receiving_packet) {
        // LOG_INFO("sample in handler: %hhu", sample);
        if (sample != SYNC && sample != NONE) {
            rx_encode_sample(ctx, sample);
        }
    }
}

void attempt_reading(ivc_context_t *ctx) {
    if (ctx->tx.flags.is_writing) {
        // LOG_INFO("CANT READ");
        return;
    }

    sample_t sample;
    if (sample_ready(&ctx->consensus, &sample)) {
        handle_sample(ctx, sample);
    }

    if (ctx->rx.flags.publish_last_rx) {
        ctx->rx.flags.publish_last_rx = false;
        LOG_INFO("packet read complete with: %hhu", ctx->rx.last_rx_value);
        ros_publish_rx(ctx->rx.last_rx_value);
        // TODO: new packet structure and acking
        //  ros_publish_rx(ctx->rx.packet_to_process & 0xFF); // trash crc
        //  if (ctx->rx.flags.awaiting_ack && ctx->rx.packet_to_process & 0xFF) {
        //      tx_enqueue_data(ctx->tx.packet_to_write_copy & 0xFF);
        //  }
        // ctx->rx.flags.awaiting_ack = false;
    }
}
