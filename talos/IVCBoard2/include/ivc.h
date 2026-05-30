#ifndef IVC_H
#define IVC_H

#include "fft/fft.h"

#include <stdbool.h>
#include <stdint.h>

#define NUM_FFT_BINS 3
// #define FFT_BIN_RANGE 500

// #define FREQ_SYNC_HZ 15000
// #define FREQ_SYNC_HZ 37000 // pinger
// #define FREQ_SYNC_HZ 51000
// #define FREQ_LOW_HZ 17000
// #define FREQ_HIGH_HZ 19000

// #define FREQ_SYNC_HZ 49000
// #define FREQ_LOW_HZ 51000
// #define FREQ_HIGH_HZ 53000

// #define FREQ_SYNC_HZ 50000
// #define FREQ_LOW_HZ 51000
// #define FREQ_HIGH_HZ 52000

// #define FREQ_SYNC_HZ 51000
// #define FREQ_LOW_HZ 61000
// #define FREQ_HIGH_HZ 71000

// #define FREQ_SYNC_HZ 51000
// #define FREQ_LOW_HZ 61000
// #define FREQ_HIGH_HZ 71000

#define FFT_BIN_RANGE 200
#define FREQ_SYNC_HZ 50500
#define FREQ_LOW_HZ 51000
#define FREQ_HIGH_HZ 51500

#define SYMBOL_PERIOD_MS 200  // lower as progress

#define CRC_POLY 0x1B  // 0b11011 (x^4 + x^3 + x + 1)
#define DATA_SIZE 8
#define CRC_SIZE 4
#define PACKET_SIZE (DATA_SIZE + CRC_SIZE)  // 1-byte MTU + 4-bit CRC

#define AMPLITUDE_IDLE_THRESHOLD 1000.0f
// #define AMPLITUDE_IDLE_THRESHOLD 50.0f  // was 40
//    #define CONSENSUS_DEPTH 5
//    #define MIN_CONSENSUS_VOTES ((CONSENSUS_DEPTH / 2) + 1)

/**
 * minimum consecutive sample observations needed
 * to determine a sample is stable
 */
#define MIN_STABLE_SAMPLES 3
/**
 * maximum amount of sample observations that are different than the most
 * recent observation needed to determine a symbol transition
 */
#define MAX_SAMPLE_GAP 2

/**
 * enumerate symbol values
 */
#define LOW ((uint8_t) 0)
#define HIGH ((uint8_t) 1)
#define SYNC ((uint8_t) 2)
#define NONE ((uint8_t) 3)
typedef uint8_t sample_t;

typedef struct {
    bool on_new_symbol;
    bool sample_ready;
    bool initialized;
} consensus_flags_t;

/**
 * values and flags needed to enable consensus/debounce
 * focused sampling
 *
 * works by waiting for a transition in sampling to determine valid symbols
 *
 * symbol transition is forced by the preceding sync symbol accompanying every 0 or 1
 */
typedef struct {
    // counts the number of sample observations that equal current_sample
    uint8_t stable_count;
    // counts the number of sample observations different than current_sample
    uint8_t gap_count;
    // the last sample found before a symbol transition
    sample_t last_sample_seen;
    // value to track the sample observation not equal to current_sample
    sample_t pending_sample;
    // the current sample that is being tracked for stability
    sample_t current_sample;
    consensus_flags_t flags;
} rx_consensus_t;  // make a was_stable field for seeing stability before disruption

/**
 * holds the buffers and swap logic for running FFT
 */
typedef struct {
    // one buffer to get ADC samples written to via DMA, one buffer for processing
    uint8_t swap_buffers[2][NSAMP];
    // buffer index for fft samples
    int8_t fft_target;
    // buffer index for processing
    bool buffer_select;
} signal_recv_t;

typedef struct {
    bool has_synced;
    bool done_writing;
    bool is_writing;
    bool can_write;
    bool need_final_sync;
} tx_flags_t;

/**
 * data relevant to processing and control flow for TX
 */
typedef struct {
    int32_t pwm_wrap_value;
    uint32_t pwm_slice_num;
    uint16_t packet_to_write_copy;  // in case of retransmission needed
    uint16_t packet_to_write;
    uint8_t data_to_write;
    uint8_t num_bits_written;
    tx_flags_t flags;
} tx_control_t;

typedef struct {
    bool receiving_packet;
    bool publish_last_rx;
    bool buffer_full;
    bool done_reading;
    bool on_new_symbol;
    bool can_read;
    bool awaiting_ack;
} rx_flags_t;

typedef struct {
    float signal[NUM_FFT_BINS];  // less resistant to noise
    float noise[NUM_FFT_BINS];   // more resistant to noise
    bool initialized;
} amplitude_ema_t;

/**
 * data relevant to processing and control flow for RX
 */
typedef struct {
    amplitude_ema_t ema;
    signal_recv_t recv;
    uint16_t current_packet;     // new, INCLUDES CRC
    uint16_t packet_to_process;  // new
    uint8_t buffer[PACKET_SIZE];
    uint8_t write_pos;
    uint8_t mtu_data;
    uint8_t crc_data;
    uint8_t last_rx_value;
    rx_flags_t flags;
} rx_control_t;

// typedef struct {
//     bool is_talos;
//     bool awaiting_ack;
// } ivc_flags_t;

/**
 * top level structure containing all relevant information
 * for the IVC board
 */
typedef struct {
    tx_control_t tx;
    rx_control_t rx;
    rx_consensus_t consensus;
    // ivc_flags_t flags;
    bool is_talos;
} ivc_context_t;

/**
 * @brief initialize ivc hardware including rx and tx
 */
void ivc_init();

/**
 * @brief tick function that will be called repeatedly by main
 *        updates consensus with a sample and attempts reading or writing
 */
void ivc_tick();

void data_ingest_tick();
void data_ingest_init();
void adc_sample_dump_tick();
void amplitude_check_tick();

#endif  // IVC_H
