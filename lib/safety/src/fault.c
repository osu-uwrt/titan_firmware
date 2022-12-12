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

        LOG_FAULT("Fault %s (%d) Raised", safety_lookup_fault_id(fault_id), fault_id);
    }
}

void safety_lower_fault(uint32_t fault_id) {
    valid_params_if(SAFETY, fault_id <= MAX_FAULT_ID);

    if ((*fault_list_reg & (1u<<fault_id)) != 0) {
        LOG_FAULT("Fault %s (%d) Lowered", safety_lookup_fault_id(fault_id), fault_id);

        // To ensure the fault led doesn't get glitched on/off due to an untimely interrupt, interrupts will be disabled during
        // the setting of the fault state and the fault LED

        uint32_t prev_interrupt_state = save_and_disable_interrupts();

        *fault_list_reg &= ~(1u<<fault_id);
        safety_set_fault_led((*fault_list_reg) != 0);

        restore_interrupts(prev_interrupt_state);
    }
}