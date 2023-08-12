#include <assert.h>
#include "driver/canbus.h"
#include "driver/led.h"
#include "titan/logger.h"

#include "dshot.h"
#include "safety_interface.h"

#undef LOGGING_UNIT_NAME
#define LOGGING_UNIT_NAME "safety_interface"

static void safety_handle_can_internal_error(canbus_error_data_t error_data) {
    LOG_ERROR("CAN Internal Error - Line: %d; Code: %d (%s Error)", error_data.error_line, error_data.error_code,
                (error_data.is_driver_error ? "Internal Driver" : "Library"));
    safety_raise_fault_with_arg(FAULT_CAN_INTERNAL_ERROR, error_data.raw);
}

// ========================================
// Implementations for External Interface Functions
// ========================================

void safety_set_fault_led(bool on) {
    canbus_set_device_in_error(on);

    led_fault_set(on);
}

void safety_handle_kill(void) {
    // Note: Any calls made in this function must be safe to be called from interrupts
    // This is because safety_kill_switch_update can be called from interrupts

    dshot_stop_thrusters();
    led_killswitch_set(false);
}

void safety_handle_enable(void) {
    led_killswitch_set(true);
}

void safety_interface_setup(void) {
    canbus_set_internal_error_cb(safety_handle_can_internal_error);
}

void safety_interface_init(void) {}

void safety_interface_tick(void) {}

void safety_interface_deinit(void) {}


// ========================================
// Constant Calculations - Does not need to be modified
// ========================================

struct kill_switch_state kill_switch_states[NUM_KILL_SWITCHES];
const int num_kill_switches = sizeof(kill_switch_states)/sizeof(*kill_switch_states);
static_assert(sizeof(kill_switch_states)/sizeof(*kill_switch_states) <= 32, "Too many kill switches defined");

const char * safety_lookup_fault_id(uint32_t fault_id) {
    return (fault_id < sizeof(fault_string_list)/sizeof(*fault_string_list) ? fault_string_list[fault_id] : "UNKNOWN");
}
