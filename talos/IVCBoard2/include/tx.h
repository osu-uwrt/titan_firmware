#ifndef TX_H
#define TX_H

#include "ivc.h"

#include <stdbool.h>
#include <stdint.h>

extern comm_state_t state;

void tx_init();
void tx_encode_bit(uint8_t bit);
void tx_encode_sync();
void tx_debug(uint8_t bit);
void tx_enqueue_data(uint8_t data);
bool tx_dequeue_data(uint8_t *data);
void tx_encode_data();
void attempt_writing();

#endif  // TX_H
