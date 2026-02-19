#ifndef RX_H
#define RX_H

#include "ivc.h"

#include <stdint.h>

#define FFT_LOW_IDX 0
#define FFT_HIGH_IDX 1
#define FFT_SYNC_IDX 2

// scale floats to chop off decimal places with an integer cast ()
#define FLOAT_SCALE_MULTIPLIER 1000.0f

extern comm_state_t state;

int8_t rx_handle_sample(float conf_value);
void rx_single_sample();
void swap_buffer_handler();
sample_t rx_sample();
void rx_init();
void handle_incoming_packet();
void attempt_packet_read();

#endif  // RX_H
