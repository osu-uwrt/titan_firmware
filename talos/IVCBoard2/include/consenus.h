#ifndef CONSENSUS_H
#define CONSENSUS_H

#include "ivc.h"

#include <stdint.h>

#define CONSENSUS_DEPTH 5

typedef struct {
    sample_t sample;
    float confidence;
} observation_t;

typedef struct {
} rx_consensus_t;

#endif  // CONSENSUS_H
