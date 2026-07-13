#ifndef CONSENSUS_H
#define CONSENSUS_H

#include "ivc.h"
#include "rx.h"

#include <stdint.h>

/**
 * @brief updates the state of the consensus engine with a sample observation
 *
 * @param c pointer to the consensus struct being used
 * @param observation sample obtained
 */
void consensus_update(rx_consensus_t *c, sample_t observation);

/**
 * @brief produces a sample if consensus can make a decision
 *
 * @param c pointer to the consensus struct being used
 * @param sample pointer to callers undefined sample where the new sample will
 *               be written to if there is a sample ready
 * @return true if a sample is ready
 *         false if no sample is ready
 */
bool sample_ready(rx_consensus_t *c, sample_t *sample);

/**
 * @brief
 */
typedef struct {
} ema_t;

#endif  // CONSENSUS_H
