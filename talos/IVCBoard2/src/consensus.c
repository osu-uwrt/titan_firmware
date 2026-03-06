#include "ivc.h"
#include "rx.h"

#include "titan/logger.h"

// void consensus_reset(rx_consensus_t *c) {
//     c->write_idx = 0;
//     c->num_samples = 0;
//     c->last_sample_seen = NONE;
//     // memset(consensus.buffer, NONE, sizeof(consensus.buffer));
// }

// bool consensus_stable(rx_consensus_t *c, sample_t *out_sample) {
//     if (c->num_samples < CONSENSUS_DEPTH) {
//         return false;
//     }

//     uint32_t num_sync_votes = 0, num_high_votes = 0, num_low_votes = 0;

//     for (uint32_t i = 0; i < CONSENSUS_DEPTH; i++) {
//         switch (c->buffer[i]) {
//         case HIGH:
//             num_high_votes++;
//             break;
//         case LOW:
//             num_low_votes++;
//             break;
//         case SYNC:
//             num_sync_votes++;
//             break;
//         }
//         // consensus.buffer[i] = NONE;
//     }

//     sample_t candidate = NONE;

//     // if (num_sync_votes >= MIN_CONSENSUS_VOTES) {
//     //     candidate = SYNC;
//     // }
//     // else if (num_high_votes >= MIN_CONSENSUS_VOTES) {
//     //     candidate = HIGH;
//     // }
//     // else if (num_low_votes >= MIN_CONSENSUS_VOTES) {
//     //     candidate = LOW;
//     // }

//     // if (candidate == NONE) {
//     //     return false;
//     // }

//     if (num_sync_votes > num_high_votes && num_sync_votes > num_low_votes) {
//         candidate = SYNC;
//     }
//     else if (num_high_votes > num_sync_votes && num_high_votes > num_low_votes) {
//         candidate = HIGH;
//     }
//     else if (num_low_votes > num_sync_votes && num_low_votes > num_high_votes) {
//         candidate = LOW;
//     }

//     // prevent repeated emissions
//     // if (candidate == consensus.last_sample_seen) {
//     //     return false;
//     // }
//     // if (candidate != consensus.last_sample_seen) {
//     //     rx.flags.on_new_symbol = true;
//     // }

//     c->last_sample_seen = candidate;
//     *out_sample = candidate;
//     return true;
// }

// =====================================================

// void consensus_push(rx_consensus_t *c, sample_t observation) {
//     // if (observation == NONE) {
//     //     return;
//     // }

//     // LOG_INFO("pushing %hhu", observation);

//     c->buffer[c->write_idx] = observation;
//     c->write_idx = (c->write_idx + 1) % CONSENSUS_DEPTH;

//     if (c->num_samples < CONSENSUS_DEPTH) {
//         c->num_samples++;
//     }

//     if (observation == c->last_sample_seen) {
//         c->stable_count++;
//     }
//     else {
//         c->gap_count++;
//     }

//     // consensus.num_samples++;
// }

void consensus_update(rx_consensus_t *c, sample_t observation) {
    if (observation == NONE)
        return;
    LOG_INFO("CONSENSUS: stable: %hhu, gap: %hhu, current sample: %hhu, last_sample_seen: %hhu, ready?: %hhu",
             c->stable_count, c->gap_count, c->current_sample, c->last_sample_seen, c->flags.sample_ready);
    if (observation == c->current_sample) {
        c->stable_count++;
        c->gap_count = 0;
    }
    else if (c->stable_count >= MIN_STABLE_SAMPLES) {
        if (observation == c->pending_sample) {
            c->gap_count++;
        }
        else {
            c->pending_sample = observation;
            c->gap_count = 1;
        }

        // c->gap_count++;
        if (c->gap_count > MAX_SAMPLE_GAP) {
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

// bool sample_is_new_symbol(rx_consensus_t *c, sample_t sample) {
//     return c->last_symbol_seen != sample;
// }

bool sample_ready(rx_consensus_t *c, sample_t *sample) {
    if (c->flags.sample_ready) {
        *sample = c->last_sample_seen;
        c->flags.sample_ready = false;
        return true;
    }
    return false;
}

sample_t get_clean_sample(rx_consensus_t *c) {
    uint32_t num_sync_votes = 0, num_high_votes = 0, num_low_votes = 0;

    for (uint32_t i = 0; i < CONSENSUS_DEPTH; i++) {
        switch (c->buffer[i]) {
        case HIGH:
            num_high_votes++;
            break;
        case LOW:
            num_low_votes++;
            break;
        case SYNC:
            num_sync_votes++;
            break;
        }
        // consensus.buffer[i] = NONE;
    }

    sample_t candidate = NONE;

    if (num_sync_votes > num_high_votes && num_sync_votes > num_low_votes) {
        candidate = SYNC;
    }
    else if (num_high_votes > num_sync_votes && num_high_votes > num_low_votes) {
        candidate = HIGH;
    }
    else if (num_low_votes > num_sync_votes && num_low_votes > num_high_votes) {
        candidate = LOW;
    }
}
