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

// #define MIN_SYMBOL_SNR 3.0f
// #define MIN_DOMINANCE 1.5f

// #define FRAME_MS (NSAMP * 1000.0f / FSAMP)
// #define FRAMES_SYMBOL (200.0f / FRAME_MS)

// #define BIN_RES (FSAMP / NSAMP)
// #define FRAME_MS (NSAMP * 1000.0f / FSAMP)
// #define FRAMES_SYMBOL (200.0f / FRAME_MS)
#define NOISE_FLOOR_EMA_ALPHA 0.15f
#define MIN_SYMBOL_SNR 2.0f
#define MIN_SYMBOL_DOMINANCE 1.0f

#define N_AVG_PINGER 5

static float pinger_avg_amp = 0.0f;
static uint32_t pinger_avg_count = 0;

static uint32_t none_observation_count = 0;
static bool timeout_started = false;

static float pinger_amp_history[N_PINGER_AMPS] = { 0 };
static uint8_t pinger_amp_history_idx = 0;

frequency_bin_t fft_bins[] = {
    { "freq_pinger", FREQ_PINGER_HZ - FFT_BIN_RANGE, FREQ_PINGER_HZ + FFT_BIN_RANGE, 0, false },
    { "ema_ref_1", FREQ_NOISE_REF_1 - FFT_BIN_RANGE, FREQ_NOISE_REF_1 + FFT_BIN_RANGE, 0, true },
    { "ema_ref_2", FREQ_NOISE_REF_2 - FFT_BIN_RANGE, FREQ_NOISE_REF_2 + FFT_BIN_RANGE, 0, true },
    { "freq_low", FREQ_LOW_HZ - FFT_BIN_RANGE, FREQ_LOW_HZ + FFT_BIN_RANGE, 0, false },
    { "freq_high", FREQ_HIGH_HZ - FFT_BIN_RANGE, FREQ_HIGH_HZ + FFT_BIN_RANGE, 0, false },
    { "freq_sync", FREQ_SYNC_HZ - FFT_BIN_RANGE, FREQ_SYNC_HZ + FFT_BIN_RANGE, 0, false },
    { "ema_ref_3", FREQ_NOISE_REF_3 - FFT_BIN_RANGE, FREQ_NOISE_REF_3 + FFT_BIN_RANGE, 0, true },
    { "ema_ref_4", FREQ_NOISE_REF_4 - FFT_BIN_RANGE, FREQ_NOISE_REF_4 + FFT_BIN_RANGE, 0, true },
};

frequency_bin_t fft_bins_pinger_20k[] = {
    { "ema_ref_1", FREQ_NOISE_REF_1_20K - FFT_BIN_RANGE, FREQ_NOISE_REF_1_20K + FFT_BIN_RANGE, 0, true },
    { "ema_ref_2", FREQ_NOISE_REF_2_20K - FFT_BIN_RANGE, FREQ_NOISE_REF_2_20K + FFT_BIN_RANGE, 0, true },
    { "freq_pinger", FREQ_PINGER_20K_HZ - FFT_BIN_RANGE, FREQ_PINGER_20K_HZ + FFT_BIN_RANGE, 0, false },
    { "ema_ref_3", FREQ_NOISE_REF_3_20K - FFT_BIN_RANGE, FREQ_NOISE_REF_3_20K + FFT_BIN_RANGE, 0, true },
    { "ema_ref_4", FREQ_NOISE_REF_4_20K - FFT_BIN_RANGE, FREQ_NOISE_REF_4_20K + FFT_BIN_RANGE, 0, true },
};
frequency_bin_t fft_bins_pinger_25k[] = {
    { "ema_ref_1", FREQ_NOISE_REF_1_25K - FFT_BIN_RANGE, FREQ_NOISE_REF_1_25K + FFT_BIN_RANGE, 0, true },
    { "ema_ref_2", FREQ_NOISE_REF_2_25K - FFT_BIN_RANGE, FREQ_NOISE_REF_2_25K + FFT_BIN_RANGE, 0, true },
    { "freq_pinger", FREQ_PINGER_25K_HZ - FFT_BIN_RANGE, FREQ_PINGER_25K_HZ + FFT_BIN_RANGE, 0, false },
    { "ema_ref_3", FREQ_NOISE_REF_3_25K - FFT_BIN_RANGE, FREQ_NOISE_REF_3_25K + FFT_BIN_RANGE, 0, true },
    { "ema_ref_4", FREQ_NOISE_REF_4_25K - FFT_BIN_RANGE, FREQ_NOISE_REF_4_25K + FFT_BIN_RANGE, 0, true },
};
frequency_bin_t fft_bins_pinger_30k[] = {
    { "ema_ref_1", FREQ_NOISE_REF_1_30K - FFT_BIN_RANGE, FREQ_NOISE_REF_1_30K + FFT_BIN_RANGE, 0, true },
    { "ema_ref_2", FREQ_NOISE_REF_2_30K - FFT_BIN_RANGE, FREQ_NOISE_REF_2_30K + FFT_BIN_RANGE, 0, true },
    { "freq_pinger", FREQ_PINGER_30K_HZ - FFT_BIN_RANGE, FREQ_PINGER_30K_HZ + FFT_BIN_RANGE, 0, false },
    { "ema_ref_3", FREQ_NOISE_REF_3_30K - FFT_BIN_RANGE, FREQ_NOISE_REF_3_30K + FFT_BIN_RANGE, 0, true },
    { "ema_ref_4", FREQ_NOISE_REF_4_30K - FFT_BIN_RANGE, FREQ_NOISE_REF_4_30K + FFT_BIN_RANGE, 0, true },
};
frequency_bin_t fft_bins_pinger_35k[] = {
    { "ema_ref_1", FREQ_NOISE_REF_1_35K - FFT_BIN_RANGE, FREQ_NOISE_REF_1_35K + FFT_BIN_RANGE, 0, true },
    { "ema_ref_2", FREQ_NOISE_REF_2_35K - FFT_BIN_RANGE, FREQ_NOISE_REF_2_35K + FFT_BIN_RANGE, 0, true },
    { "freq_pinger", FREQ_PINGER_35K_HZ - FFT_BIN_RANGE, FREQ_PINGER_35K_HZ + FFT_BIN_RANGE, 0, false },
    { "ema_ref_3", FREQ_NOISE_REF_3_35K - FFT_BIN_RANGE, FREQ_NOISE_REF_3_35K + FFT_BIN_RANGE, 0, true },
    { "ema_ref_4", FREQ_NOISE_REF_4_35K - FFT_BIN_RANGE, FREQ_NOISE_REF_4_35K + FFT_BIN_RANGE, 0, true },
};

frequency_bin_t fft_bins_pinger_37k[] = {
    { "ema_ref_1", FREQ_NOISE_REF_1_37K - FFT_BIN_RANGE, FREQ_NOISE_REF_1_37K + FFT_BIN_RANGE, 0, true },
    { "ema_ref_2", FREQ_NOISE_REF_2_37K - FFT_BIN_RANGE, FREQ_NOISE_REF_2_37K + FFT_BIN_RANGE, 0, true },
    { "freq_pinger", FREQ_PINGER_37K_HZ - FFT_BIN_RANGE, FREQ_PINGER_37K_HZ + FFT_BIN_RANGE, 0, false },
    { "ema_ref_3", FREQ_NOISE_REF_3_37K - FFT_BIN_RANGE, FREQ_NOISE_REF_3_37K + FFT_BIN_RANGE, 0, true },
    { "ema_ref_4", FREQ_NOISE_REF_4_37K - FFT_BIN_RANGE, FREQ_NOISE_REF_4_37K + FFT_BIN_RANGE, 0, true },
};
frequency_bin_t fft_bins_pinger_40k[] = {
    { "ema_ref_1", FREQ_NOISE_REF_1_40K - FFT_BIN_RANGE, FREQ_NOISE_REF_1_40K + FFT_BIN_RANGE, 0, true },
    { "ema_ref_2", FREQ_NOISE_REF_2_40K - FFT_BIN_RANGE, FREQ_NOISE_REF_2_40K + FFT_BIN_RANGE, 0, true },
    { "freq_pinger", FREQ_PINGER_40K_HZ - FFT_BIN_RANGE, FREQ_PINGER_40K_HZ + FFT_BIN_RANGE, 0, false },
    { "ema_ref_3", FREQ_NOISE_REF_3_40K - FFT_BIN_RANGE, FREQ_NOISE_REF_3_40K + FFT_BIN_RANGE, 0, true },
    { "ema_ref_4", FREQ_NOISE_REF_4_40K - FFT_BIN_RANGE, FREQ_NOISE_REF_4_40K + FFT_BIN_RANGE, 0, true },
};

frequency_bin_t *pinger_bin_map[] = {
    [FREQ_20KHZ] = fft_bins_pinger_20k, [FREQ_25KHZ] = fft_bins_pinger_25k, [FREQ_30KHZ] = fft_bins_pinger_30k,
    [FREQ_35KHZ] = fft_bins_pinger_35k, [FREQ_37KHZ] = fft_bins_pinger_37k, [FREQ_40KHZ] = fft_bins_pinger_40k,
};

// frequency_bin_t fft_bins[] = {
//     { "ema_ref_1", FREQ_NOISE_REF_1 - FFT_BIN_RANGE, FREQ_NOISE_REF_1 + FFT_BIN_RANGE, 0, true },
//     { "ema_ref_2", FREQ_NOISE_REF_2 - FFT_BIN_RANGE, FREQ_NOISE_REF_2 + FFT_BIN_RANGE, 0, true },
//     { "freq_sync", FREQ_SYNC_HZ - FFT_BIN_RANGE, FREQ_SYNC_HZ + FFT_BIN_RANGE, 0, false },
//     { "freq_low", FREQ_LOW_HZ - FFT_BIN_RANGE, FREQ_LOW_HZ + FFT_BIN_RANGE, 0, false },
//     { "freq_high", FREQ_HIGH_HZ - FFT_BIN_RANGE, FREQ_HIGH_HZ + FFT_BIN_RANGE, 0, false },
//     { "ema_ref_3", FREQ_NOISE_REF_3 - FFT_BIN_RANGE, FREQ_NOISE_REF_3 + FFT_BIN_RANGE, 0, true },
//     { "ema_ref_4", FREQ_NOISE_REF_4 - FFT_BIN_RANGE, FREQ_NOISE_REF_4 + FFT_BIN_RANGE, 0, true },
// };

bool push_pinger_amp(float amp) {
    pinger_amp_history[pinger_amp_history_idx] = amp;
    pinger_amp_history_idx++;
    if (pinger_amp_history_idx >= N_AVG_PINGER) {
        pinger_amp_history_idx = 0;
        return true;
    }
    return false;
}

float get_max_pinger_amp() {
    float max = 0.0f;
    for (uint8_t i = 0; i < N_PINGER_AMPS; i++) {
        if (pinger_amp_history[i] > max) {
            max = pinger_amp_history[i];
        }
    }
    return max;
}

void rx_init(ivc_context_t *ctx, void (*swap_buffer_handler)()) {
    // set the callback that fires when dma fills a swap buffer with data
    fft_setup(swap_buffer_handler, &ctx->rx.flags.is_pinger_mode, &ctx->rx.pinger_mode);
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

static sample_t detect_symbol(float snr_low, float snr_high, float snr_sync) {
    bool low_present = snr_low >= MIN_SYMBOL_SNR;
    bool high_present = snr_high >= MIN_SYMBOL_SNR;
    bool sync_present = snr_sync >= MIN_SYMBOL_SNR;

    // nothing clears threshold
    if (!low_present && !high_present && !sync_present)
        return NONE;

    // find the dominant tone
    float best = fmaxf(snr_sync, fmaxf(snr_low, snr_high));

    // winner must clearly dominate all others
    if (best == snr_sync) {
        if (snr_sync / (snr_low + 1e-6f) >= MIN_SYMBOL_DOMINANCE &&
            snr_sync / (snr_high + 1e-6f) >= MIN_SYMBOL_DOMINANCE)
            return SYNC;
    }
    else if (best == snr_low) {
        if (snr_low / (snr_sync + 1e-6f) >= MIN_SYMBOL_DOMINANCE &&
            snr_low / (snr_high + 1e-6f) >= MIN_SYMBOL_DOMINANCE)
            return LOW;
    }
    else {
        if (snr_high / (snr_sync + 1e-6f) >= MIN_SYMBOL_DOMINANCE &&
            snr_high / (snr_low + 1e-6f) >= MIN_SYMBOL_DOMINANCE)
            return HIGH;
    }

    return NONE;  // ambiguous
}

sample_t rx_observe_3(ivc_context_t *ctx) {
    fft_process(ctx->rx.recv.swap_buffers[ctx->rx.recv.fft_target], fft_bins, NUM_FFT_BINS);
    float noise_floor;
    if (fft_get_noise_floor(&noise_floor)) {
        float low_snr = fft_bins[FFT_LOW_IDX].amplitude / (noise_floor + 1e-6f);
        float high_snr = fft_bins[FFT_HIGH_IDX].amplitude / (noise_floor + 1e-6f);
        float sync_snr = fft_bins[FFT_SYNC_IDX].amplitude / (noise_floor + 1e-6f);
        // LOG_INFO("\nNOISE FLOOR: %0.5f\n", noise_floor);
        // LOG_INFO("\nSNR RATIOS:\nLOW: %0.5f\nHIGH:%0.5f\nSYNC: %0.5f\n", low_snr, high_snr, sync_snr);
        sample_t sample = detect_symbol(low_snr, high_snr, sync_snr);
        // LOG_INFO("SAMPLE OBTAINED IN OBSERVATION: %hhu\n", sample);
        return sample;
    }
    return NONE;
}

// bool rx_observe_pinger(ivc_context_t *ctx, float *avg) {
//     fft_process(ctx->rx.recv.swap_buffers[ctx->rx.recv.fft_target], fft_bins, NUM_FFT_BINS);
//     if (pinger_avg_count < N_AVG_PINGER) {
//         pinger_avg_amp += fft_bins[FFT_PINGER_IDX].amplitude;
//         pinger_avg_count++;
//         return false;
//     }
//     else {
//         pinger_avg_count = 0;
//         *avg = pinger_avg_amp / N_AVG_PINGER;
//         pinger_avg_amp = 0.0f;
//         return true;
//     }
// }

bool rx_observe_pinger(ivc_context_t *ctx, float *avg) {
    frequency_bin_t *bins = pinger_bin_map[ctx->rx.pinger_mode];
    fft_process(ctx->rx.recv.swap_buffers[ctx->rx.recv.fft_target], bins, NUM_PINGER_FFT_BINS);
    if (pinger_avg_count < N_AVG_PINGER) {
        pinger_avg_amp += bins[FFT_PINGER_SIGNAL_IDX].amplitude;
        pinger_avg_count++;
        return false;
    }
    else {
        pinger_avg_count = 0;
        *avg = pinger_avg_amp / N_AVG_PINGER;
        pinger_avg_amp = 0.0f;
        return true;
    }
}

void rx_handle_pinger(ivc_context_t *ctx) {
    frequency_bin_t *bins = pinger_bin_map[ctx->rx.pinger_mode];
    fft_process(ctx->rx.recv.swap_buffers[ctx->rx.recv.fft_target], bins, NUM_PINGER_FFT_BINS);
    float amp = bins[FFT_PINGER_SIGNAL_IDX].amplitude;
    if (push_pinger_amp(amp)) {
        ros_publish_pinger_amp(get_max_pinger_amp());
    }
}

// sample_t rx_observe_2(ivc_context_t *ctx) {
//     ema_update(ctx->rx.ema, fft_bins);

//     float high_snr = ema_snr(ctx->rx.ema, FFT_HIGH_IDX);
//     float low_snr = ema_snr(ctx->rx.ema, FFT_LOW_IDX);
//     float sync_snr = ema_snr(ctx->rx.ema, FFT_SYNC_IDX);

//     return detect_symbol(low_snr, high_snr, sync_snr);
// }

float rx_observe_amplitude(ivc_context_t *ctx) {
    // fft_process(recv_data.swap_buffers[recv_data.fft_target], fft_bins, NUM_FFT_BINS);
    fft_process(ctx->rx.recv.swap_buffers[ctx->rx.recv.fft_target], fft_bins, NUM_FFT_BINS);
    return fft_bins[FFT_SYNC_IDX].amplitude;
}

float get_low_amp() {
    return fft_bins[FFT_LOW_IDX].amplitude;
}

float get_sync_amp() {
    return fft_bins[FFT_SYNC_IDX].amplitude;
}

float get_high_amp() {
    return fft_bins[FFT_HIGH_IDX].amplitude;
}

float get_ref1_amp() {
    return fft_bins[FFT_REF1_IDX].amplitude;
}

float get_ref2_amp() {
    return fft_bins[FFT_REF2_IDX].amplitude;
}

float get_ref3_amp() {
    return fft_bins[FFT_REF3_IDX].amplitude;
}

float get_ref4_amp() {
    return fft_bins[FFT_REF4_IDX].amplitude;
}

float get_pinger_amp() {
    return fft_bins[FFT_PINGER_IDX].amplitude;
}

void process_fft(ivc_context_t *ctx) {
    fft_process(ctx->rx.recv.swap_buffers[ctx->rx.recv.fft_target], fft_bins, NUM_FFT_BINS);
}

// bool accumulate_fft_window(ivc_context_t *ctx) {
//     fft_window_accumulate_t *accum = &ctx->rx.window_accum;

//     if (accum->counter == N_ACCUMULATE) {
//         accum->counter = 0;
//         for (int i = FFT_LOW_IDX; i <= FFT_SYNC_IDX; i++) {
//             accum->accum_buffer[i] /= N_ACCUMULATE;
//         }
//         return true;
//     }
//     else {
//         fft_process(ctx->rx.recv.swap_buffers[ctx->rx.recv.fft_target], fft_bins, NUM_FFT_BINS);
//         for (int i = FFT_LOW_IDX; i <= FFT_SYNC_IDX; i++) {
//             accum->accum_buffer[i] += fft_bins[i].amplitude;
//         }
//         accum->counter++;
//         return false;
//     }
// }

sample_t rx_observe(ivc_context_t *ctx) {
    // fft_process(recv_data.swap_buffers[recv_data.fft_target], fft_bins, NUM_FFT_BINS);
    fft_process(ctx->rx.recv.swap_buffers[ctx->rx.recv.fft_target], fft_bins, NUM_FFT_BINS);
    float energy_at_low = fft_bins[FFT_LOW_IDX].amplitude;
    float energy_at_high = fft_bins[FFT_HIGH_IDX].amplitude;
    float energy_at_sync = fft_bins[FFT_SYNC_IDX].amplitude;
    LOG_INFO("\nlow: %f\nhigh: %f\nsync: %f\n", energy_at_low, energy_at_high, energy_at_sync);

    // if (energy_at_sync > 400.0f) {
    //     // LOG_INFO("sync was: %04f", energy_at_sync);
    //     // LOG_INFO("sync greater than threshold");
    //     LOG_INFO("\nSAW THE PINGERRRRRRRRRRR WITH %f\n", energy_at_sync);
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
        LOG_INFO("RECEIVING PACKET\n");
    }
    else if (ctx->rx.flags.receiving_packet) {
        // LOG_INFO("sample in handler: %hhu", sample);
        if (sample != SYNC && sample != NONE) {
            rx_encode_sample(ctx, sample);
        }

        // if (sample == NONE) {
        //     none_observation_count++;
        // }

        // if (none_observation_count >= IDLE_TIMEOUT) {
        //     LOG_INFO("CONNECTION TIMED OUT\n");
        //     none_observation_count = 0;
        //     rx_reset(ctx);
        // }
    }
}

// void attempt_reading(ivc_context_t *ctx) {
//     if (ctx->tx.flags.is_writing) {
//         // LOG_INFO("CANT READ");
//         return;
//     }

//     sample_t sample;
//     if (sample_ready(&ctx->consensus, &sample)) {
//         LOG_INFO("SAMPLE FOUND: %hhu", sample);
//         handle_sample(ctx, sample);
//     }

//     if (ctx->rx.flags.publish_last_rx) {
//         ctx->rx.flags.publish_last_rx = false;
//         LOG_INFO("packet read complete with: %hhu", ctx->rx.last_rx_value);
//         ros_publish_rx(ctx->rx.last_rx_value);
//         // TODO: new packet structure and acking
//         //  ros_publish_rx(ctx->rx.packet_to_process & 0xFF); // trash crc
//         //  if (ctx->rx.flags.awaiting_ack && ctx->rx.packet_to_process & 0xFF) {
//         //      tx_enqueue_data(ctx->tx.packet_to_write_copy & 0xFF);
//         //  }
//         // ctx->rx.flags.awaiting_ack = false;
//     }
// }

void attempt_reading(ivc_context_t *ctx) {
    if (ctx->tx.flags.is_writing) {
        // LOG_INFO("CANT READ");
        return;
    }

    sample_t sample;
    if (sample_ready(&ctx->consensus, &sample)) {
        // LOG_INFO("\nSAMPLE FOUND: %hhu\n", sample);
        handle_sample(ctx, sample);
    }

    if (ctx->rx.flags.publish_last_rx) {
        ctx->rx.flags.publish_last_rx = false;
        LOG_INFO("\n\npacket read complete with: %hhu\n\n", ctx->rx.last_rx_value);
        ros_publish_rx(ctx->rx.last_rx_value);
        ctx->rx.flags.receiving_packet = false;
        // TODO: new packet structure and acking
        //  ros_publish_rx(ctx->rx.packet_to_process & 0xFF); // trash crc
        //  if (ctx->rx.flags.awaiting_ack && ctx->rx.packet_to_process & 0xFF) {
        //      tx_enqueue_data(ctx->tx.packet_to_write_copy & 0xFF);
        //  }
        // ctx->rx.flags.awaiting_ack = false;
    }
}

void rx_timeout_start() {
    if (!timeout_started) {
        timeout_started = true;
        none_observation_count = 0;
    }
}

void rx_timeout_advance() {
    none_observation_count++;
}

bool rx_timeout_expired() {
    if (none_observation_count >= 5) {
        return true;
    }
    return false;
}

void rx_timeout(ivc_context_t *ctx) {
    timeout_started = false;
    rx_reset(ctx);
}

// void attempt_reading() {}
