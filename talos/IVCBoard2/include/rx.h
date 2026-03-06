#ifndef RX_H
#define RX_H

#include "ivc.h"

#include <stdint.h>

#define FFT_LOW_IDX 0
#define FFT_HIGH_IDX 1
#define FFT_SYNC_IDX 2

// scale floats to chop off decimal places with an integer cast ()
#define FLOAT_SCALE_MULTIPLIER 1000.0f

// #define CONSENSUS_DEPTH 5
// #define MIN_CONSENSUS_VOTES ((CONSENSUS_DEPTH / 2) + 1)
// // #define MIN_STABLE_SAMPLES ((CONSENSUS_DEPTH / 2) + 1)
// #define MIN_STABLE_SAMPLES 3
// #define MAX_SAMPLE_GAP 2

// typedef struct {
//     uint8_t high;
//     uint8_t low;
//     uint8_t sync;
// } votes_t;

// typedef struct {
//     bool on_new_symbol;
//     bool sample_ready;
//     bool initialized;
// } consensus_flags_t;

// typedef struct {
//     votes_t votes;
//     sample_t buffer[CONSENSUS_DEPTH];
//     sample_t last_sample_seen;
//     uint8_t num_samples;
//     uint8_t write_idx;

//     uint8_t stable_count;
//     uint8_t gap_count;
//     sample_t pending_sample;
//     sample_t previous_sample;
//     sample_t current_sample;
//     consensus_flags_t flags;
// } rx_consensus_t;

// int8_t rx_handle_sample(float conf_value);
// void rx_single_sample();
// void swap_buffer_handler();
// sample_t rx_sample();
// void rx_init();
// void handle_incoming_packet();
// void attempt_packet_read();
// void rx_encode_sample(sample_t sample);
//
// // consensus
// sample_t rx_observe();
// void consensus_push(sample_t observation);
// void consensus_push_fast(sample_t observation);
// bool consensus_stable(sample_t *out_sample);
// bool consensus_stable_fast(sample_t *out_sample);
// void consensus_reset_fast();
// sample_t new_rx_sample();
// void new_attempt_packet_read();
//
// void listen_for_packet();
// void listen_for_packet_no_timer();
//
// void read_packet();
// void rx_try_write(sample_t sample);
// void rx_try_reset();
//
// void unlock_reading(rx_control_t *rx);
// void lock_reading(rx_control_t *rx);
// void handle_sample(ivc_context_t *ctx, sample_t sample);
// void attempt_reading(ivc_context_t *ctx);
// bool is_listening(rx_control_t *rx);

void handle_sample(ivc_context_t *ctx, sample_t sample);
void attempt_reading(ivc_context_t *ctx);
void rx_reset(ivc_context_t *ctx);
void rx_encode_sample(ivc_context_t *ctx, sample_t sample);
sample_t rx_observe(ivc_context_t *ctx);
bool is_idle(float e0, float e1, float e2);
sample_t get_sample(float conf_value);
void rx_init(ivc_context_t *ctx, void (*swap_buffer_handler)());

#endif  // RX_H
