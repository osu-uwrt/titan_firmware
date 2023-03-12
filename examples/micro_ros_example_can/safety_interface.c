#include <assert.h>

#include "can_pio/canbus.h"
#include "safety_interface.h"


void safety_handle_can_internal_error(__unused canbus_error_data_t error_data) {
    safety_raise_fault(FAULT_CAN_INTERNAL_ERROR);
}

void safety_handle_can_receive_error(__unused enum canbus_receive_error_codes err_code) {
    safety_raise_fault(FAULT_CAN_RECV_ERROR);
}


// ========================================
// Implementations for External Interface Functions
// ========================================

void safety_set_fault_led(bool on) {
    canbus_set_device_in_error(on);
}

void safety_handle_kill(void) {
    // Note: Any calls made in this function must be safe to be called from interrupts
    // This is because safety_kill_switch_update can be called from interrupts

    // TODO: Modify this function to add callbacks when system is killed
}

void safety_handle_enable(void) {
    // TODO: Modify this function to add callbacks for when system is enabled
}

void safety_interface_setup(void) {
    canbus_set_receive_error_cb(safety_handle_can_receive_error);
    canbus_set_internal_error_cb(safety_handle_can_internal_error);
}

void safety_interface_init(void) {
    // TODO: Modify this function to add code to be called during safety_init
}

void safety_interface_tick(void) {
    if (canbus_initialized && canbus_core_dead()) {
        safety_raise_fault(FAULT_CAN_INTERNAL_ERROR);
    }
}

void safety_interface_deinit(void) {
    // TODO: Modify this function to add code to be called during safety_deinit
}


// ========================================
// Constant Calculations - Does not need to be modified
// ========================================

struct kill_switch_state kill_switch_states[NUM_KILL_SWITCHES];
const int num_kill_switches = sizeof(kill_switch_states)/sizeof(*kill_switch_states);
static_assert(sizeof(kill_switch_states)/sizeof(*kill_switch_states) <= 32, "Too many kill switches defined");

const char * safety_lookup_fault_id(uint32_t fault_id) {
    return (fault_id < sizeof(fault_string_list)/sizeof(*fault_string_list) ? fault_string_list[fault_id] : "UNKNOWN");
}