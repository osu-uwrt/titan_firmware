#include "safety_internal.h"

#include "hardware/sync.h"
#include "pico/assert.h"

#include <stdbool.h>
#include <stdint.h>

// ========================================
// Fault Management Functions
// ========================================

static spin_lock_t *fault_lock;
struct fault_data safety_fault_data[MAX_FAULT_ID + 1] = { 0 };

void safety_raise_fault_full(uint32_t fault_id, uint32_t arg, const char *filename, uint16_t line) {
    valid_params_if(SAFETY, fault_id <= MAX_FAULT_ID);

    absolute_time_t raise_time = get_absolute_time();

    uint32_t prev_interrupt_state = spin_lock_blocking(fault_lock);
    safety_fault_data[fault_id].time = raise_time;
    safety_fault_data[fault_id].extra_data = arg;
    safety_fault_data[fault_id].filename = filename;
    safety_fault_data[fault_id].line = line;
    safety_fault_data[fault_id].sticky_fault = true;

    if ((*fault_list_reg & (1u << fault_id)) == 0) {
        // To ensure the fault led doesn't get glitched on/off due to an untimely interrupt, interrupts will be disabled
        // during the setting of the fault state and the fault LED

        *fault_list_reg |= (1 << fault_id);
    }
    else {
        safety_fault_data[fault_id].multiple_fires = true;
    }

    spin_unlock(fault_lock, prev_interrupt_state);
}

void safety_lower_fault(uint32_t fault_id) {
    valid_params_if(SAFETY, fault_id <= MAX_FAULT_ID);

    if ((*fault_list_reg & (1u << fault_id)) != 0) {
        // To ensure the fault led doesn't get glitched on/off due to an untimely interrupt, interrupts will be disabled
        // during the setting of the fault state and the fault LED

        uint32_t prev_interrupt_state = spin_lock_blocking(fault_lock);
        *fault_list_reg &= ~(1u << fault_id);
        spin_unlock(fault_lock, prev_interrupt_state);
    }
}

void safety_internal_fault_setup(void) {
    fault_lock = spin_lock_init(spin_lock_claim_unused(true));
}

void safety_internal_fault_tick(void) {
    static uint32_t last_fault_value = 0;
    uint32_t fault_list = *fault_list_reg;
    safety_set_fault_led(fault_list != 0);

    uint32_t outstanding_faults = fault_list ^ last_fault_value;

    last_fault_value = fault_list;
    int i = 0;

    while (outstanding_faults) {
        // Check current list
        if (outstanding_faults & 1) {
            if (fault_list & 1) {
                // Fault was not present in last tick
                LOG_FAULT("Fault %s (%d) Raised (Arg: 0x%08lX, Location: %s:%d)", safety_lookup_fault_id(i), i,
                          safety_fault_data[i].extra_data, safety_fault_data[i].filename, safety_fault_data[i].line);
            }
            else {
                // Fault was present and is no longer present
                LOG_FAULT("Fault %s (%d) Lowered", safety_lookup_fault_id(i), i);
            }
        }

        // Check next fault
        i++;
        fault_list >>= 1;
        outstanding_faults >>= 1;
    }
}
