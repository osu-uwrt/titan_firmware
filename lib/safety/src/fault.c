#include <stdbool.h>
#include <stdint.h>

#include "pico/assert.h"
#include "hardware/sync.h"

#include "safety_internal.h"

// ========================================
// Fault Management Functions
// ========================================

void safety_raise_fault(uint32_t fault_id) {
    valid_params_if(SAFETY, fault_id <= MAX_FAULT_ID);

    if ((*fault_list_reg & (1u<<fault_id)) == 0) {
        // To ensure the fault led doesn't get glitched on/off due to an untimely interrupt, interrupts will be disabled during
        // the setting of the fault state and the fault LED

        uint32_t prev_interrupt_state = save_and_disable_interrupts();

        *fault_list_reg |= (1<<fault_id);
        safety_set_fault_led(true);

        restore_interrupts(prev_interrupt_state);
    }
}

void safety_lower_fault(uint32_t fault_id) {
    valid_params_if(SAFETY, fault_id <= MAX_FAULT_ID);

    if ((*fault_list_reg & (1u<<fault_id)) != 0) {
        // To ensure the fault led doesn't get glitched on/off due to an untimely interrupt, interrupts will be disabled during
        // the setting of the fault state and the fault LED

        uint32_t prev_interrupt_state = save_and_disable_interrupts();

        *fault_list_reg &= ~(1u<<fault_id);
        safety_set_fault_led((*fault_list_reg) != 0);

        restore_interrupts(prev_interrupt_state);
    }
}

void safety_internal_fault_tick(void) {
    static uint32_t last_fault_value = 0;
    uint32_t fault_list = *fault_list_reg;
    uint32_t outstanding_faults = fault_list ^ last_fault_value;

    last_fault_value = fault_list;
    int i = 0;

    while (outstanding_faults) {
        // Check current list
        if (outstanding_faults & 1) {
            if (fault_list & 1) {
                // Fault was not present in last tick
                LOG_FAULT("Fault %s (%d) Raised", safety_lookup_fault_id(i), i);
            } else {
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