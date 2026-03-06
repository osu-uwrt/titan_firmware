#ifndef CONSENSUS_H
#define CONSENSUS_H

#include "ivc.h"
#include "rx.h"

#include <stdint.h>

void consensus_update(rx_consensus_t *c, sample_t observation);
bool sample_ready(rx_consensus_t *c, sample_t *sample);

#endif  // CONSENSUS_H
