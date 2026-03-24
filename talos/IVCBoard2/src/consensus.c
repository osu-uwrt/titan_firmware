#include "consensus.h"

#include "ivc.h"
#include "rx.h"

#include "titan/logger.h"

void consensus_update(rx_consensus_t *c, sample_t observation) {
    if (observation == NONE)
        return;
    // LOG_INFO("CONSENSUS: stable: %hhu, gap: %hhu, current sample: %hhu, last_sample_seen: %hhu, ready?: %hhu",
    //  c->stable_count, c->gap_count, c->current_sample, c->last_sample_seen, c->flags.sample_ready);
    if (observation == c->current_sample) {
        LOG_INFO("STABLE COUNT: %hhu", c->stable_count);
        c->stable_count++;
        c->gap_count = 0;
    }
    else if (c->stable_count >= MIN_STABLE_SAMPLES) {
        if (observation == c->pending_sample) {
            LOG_INFO("OBS MATCHES PENDING, GAP = %hhu", c->gap_count);
            c->gap_count++;
        }
        else {
            LOG_INFO("OBS DOESNT MATCH PENDING");
            c->pending_sample = observation;
            c->gap_count = 1;
        }

        // c->gap_count++;
        if (c->gap_count >= MAX_SAMPLE_GAP) {
            LOG_INFO("TRANSITIONING SYMBOL");
            c->last_sample_seen = c->current_sample;
            c->current_sample = c->pending_sample;
            c->stable_count = 1;
            c->gap_count = 0;
            c->flags.sample_ready = true;
        }
    }
    else {
        c->stable_count = 1;
        c->gap_count = 0;
        c->current_sample = observation;
        c->pending_sample = observation;
    }
}

bool sample_ready(rx_consensus_t *c, sample_t *sample) {
    if (c->flags.sample_ready) {
        LOG_INFO("sample ready: %hhu", c->last_sample_seen);
        *sample = c->last_sample_seen;
        c->flags.sample_ready = false;
        return true;
    }
    return false;
}
