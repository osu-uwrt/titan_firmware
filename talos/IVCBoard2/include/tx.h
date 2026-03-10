#ifndef TX_H
#define TX_H

#include "ivc.h"

#include <stdbool.h>
#include <stdint.h>

/**
 * @brief will attempt to write out a queued packet
 *
 * @param ctx pointer to the main context struct
 */
void attempt_writing(ivc_context_t *ctx);

/**
 * @brief enqueues data that will be transmitted over tx
 *
 * @param data the data to be transmitted
 */
void tx_enqueue_data(uint8_t data);

/**
 * @brief initalize tx hardware
 *
 * @param ctx pointer to the main context struct
 */
void tx_init(ivc_context_t *ctx);

/**
 * @brief function to debug tx with a single frequency transmission
 *
 * @param ctx pointer to the main context struct
 * @param bit the bit to be transmitted
 */
void tx_debug(ivc_context_t *ctx, uint8_t bit);

#endif  // TX_H
