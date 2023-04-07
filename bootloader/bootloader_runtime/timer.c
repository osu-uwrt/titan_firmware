/*
 * Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "hardware/timer.h"

/// tag::time_us_64[]
uint64_t time_us_64() {
    uint32_t lo = timer_hw->timelr;
    uint32_t hi = timer_hw->timehr;
    return ((uint64_t) hi << 32u) | lo;
}
/// end::time_us_64[]

/// \tag::busy_wait[]
void busy_wait_us_32(uint32_t delay_us) {
    if (0 <= (int32_t)delay_us) {
        // we only allow 31 bits, otherwise we could have a race in the loop below with
        // values very close to 2^32
        uint32_t start = timer_hw->timerawl;
        while (timer_hw->timerawl - start < delay_us) {
            tight_loop_contents();
        }
    } else {
        busy_wait_us(delay_us);
    }
}

void busy_wait_us(uint64_t delay_us) {
    uint64_t base = time_us_64();
    uint64_t target = base + delay_us;
    if (target < base) {
        target = (uint64_t)-1;
    }
    absolute_time_t t;
    update_us_since_boot(&t, target);
    busy_wait_until(t);
}

void busy_wait_ms(uint32_t delay_ms)
{
    if (delay_ms <= 0x7fffffffu / 1000) {
        busy_wait_us_32(delay_ms * 1000);
    } else {
        busy_wait_us(delay_ms * 1000ull);
    }
}

void busy_wait_until(absolute_time_t t) {
    uint64_t target = to_us_since_boot(t);
    uint32_t hi_target = (uint32_t)(target >> 32u);
    uint32_t hi = timer_hw->timerawh;
    while (hi < hi_target) {
        hi = timer_hw->timerawh;
        tight_loop_contents();
    }
    while (hi == hi_target && timer_hw->timerawl < (uint32_t) target) {
        hi = timer_hw->timerawh;
        tight_loop_contents();
    }
}
/// \end::busy_wait[]
