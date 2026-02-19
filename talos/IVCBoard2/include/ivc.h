#ifndef IVC_H
#define IVC_H

#include "fft/fft.h"

#include <stdbool.h>
#include <stdint.h>

#define NUM_FFT_BINS 3
#define FFT_BIN_RANGE 500

#define FREQ_SYNC_HZ 15000
#define FREQ_LOW_HZ 17000
#define FREQ_HIGH_HZ 19000

#define SYMBOL_PERIOD_MS 200  // lower as progress

#define CRC_POLY 0x1B  // 0b11011 (x^4 + x^3 + x + 1)
#define DATA_SIZE 8
#define CRC_SIZE 4
#define PACKET_SIZE (DATA_SIZE + CRC_SIZE)  // 1-byte MTU + 4-bit CRC

#define AMPLITUDE_IDLE_THRESHOLD 300.0f

#define LOW ((uint8_t) 0)
#define HIGH ((uint8_t) 1)
#define SYNC ((uint8_t) 2)
#define NONE ((uint8_t) 3)
typedef uint8_t sample_t;

// typedef enum {
//     LOW,
//     HIGH,
//     SYNC,
//     NONE,
// } sample_t;

typedef struct {
    uint8_t buffer[7];
    uint8_t correct_sample;
} consensus_t;

// typedef struct {
//     uint8_t buffers[2][NSAMP];
//     int8_t fft_target;
//     bool buffer_select;
// } sampling_t;

typedef struct {
    uint8_t swap_buffers[2][NSAMP];
    int8_t fft_target;
    bool buffer_select;
} signal_recv_t;

typedef struct {
    int32_t pwm_wrap_value;
    uint32_t pwm_slice_num;
    uint8_t data_to_write;
    uint8_t num_bits_written;
    bool has_synced;
    bool done_writing;
    bool is_writing;
} tx_control_t;

typedef struct {
    uint8_t prev_rx_val;
    bool packet_in_flight;
    bool receiving_packet;
} rx_control_t;

typedef struct {
    uint8_t buffer[PACKET_SIZE];
    uint8_t write_pos;
    uint8_t mtu_data;
    uint8_t last_rx_val;
    bool buffer_full;
} rx_capture_t;

typedef struct {
    uint8_t data;
    bool is_sync_ping;
} tx_data_t;

// typedef struct {
//     bool packet_in_flight;
//     bool link_established;
//     bool sync_found;
// } comm_state_t;

// 001 = packet_in_flight, 010 = link_established, 100 = sync_found
// typedef uint8_t comm_state_t;
typedef enum {
    IDLE,
    PACKET_IN_FLIGHT,
    LINK_ESTABLISHED,
    SYNC_FOUND,
    TRANSMITTING,
    RECEIVING,
} comm_state_t;

void ivc_init();
uint8_t calculate_crc(uint8_t packet);
void tick();

#endif  // IVC_H
