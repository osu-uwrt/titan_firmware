#ifndef RX_H
#define RX_H

#include "ivc.h"

#include <stdint.h>

#define FFT_LOW_IDX 0
#define FFT_HIGH_IDX 1
#define FFT_SYNC_IDX 2

// scale floats to chop off decimal places with an integer cast ()
#define FLOAT_SCALE_MULTIPLIER 1000.0f

#define CONSENSUS_DEPTH 50
#define MIN_CONSENSUS_VOTES ((CONSENSUS_DEPTH / 2) + 1)

extern comm_state_t state;

typedef struct {
    uint8_t high;
    uint8_t low;
    uint8_t sync;
} votes_t;

typedef struct {
    votes_t votes;
    sample_t buffer[CONSENSUS_DEPTH];
    sample_t last_sample_seen;
    uint8_t num_samples;
    uint8_t write_idx;
    uint8_t symbol_ticks;
} rx_consensus_t;

typedef struct {
    uint32_t ticks;
} symbol_clock_t;

int8_t rx_handle_sample(float conf_value);
void rx_single_sample();
void swap_buffer_handler();
sample_t rx_sample();
void rx_init();
void handle_incoming_packet();
void attempt_packet_read();

// consensus
sample_t rx_observe();
void consensus_push(sample_t observation);
void consensus_push_fast(sample_t observation);
bool consensus_stable(sample_t *out_sample);
bool consensus_stable_fast(sample_t *out_sample);
sample_t new_rx_sample();
void new_attempt_packet_read();

void listen_for_packet();

#endif  // RX_H
