#ifndef TX_H
#define TX_H

#include "ivc.h"

#include <stdbool.h>
#include <stdint.h>

//
// void tx_init();
// void tx_encode_bit(uint8_t bit);
// void tx_encode_sync();
// void tx_debug(uint8_t bit);
// void tx_enqueue_data(uint8_t data);
// bool tx_dequeue_data(uint8_t *data);
// void tx_encode_data();
// void attempt_writing();
// void unlock_writing(tx_control_t *tx);
// void lock_writing(tx_control_t *tx);

void attempt_writing(ivc_context_t *ctx);
bool tx_dequeue_data(uint8_t *data);
void tx_enqueue_data(uint8_t data);
void tx_encode_sync(ivc_context_t *ctx);
void tx_encode_bit(ivc_context_t *ctx, uint8_t bit);
void tx_init(ivc_context_t *ctx);
void tx_debug(ivc_context_t *ctx, uint8_t bit);

#endif  // TX_H
