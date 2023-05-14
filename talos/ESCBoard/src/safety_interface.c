#include <assert.h>

#include "driver/canbus.h"
#include "driver/led.h"

#include "dshot.h"
#include "safety_interface.h"

void safety_handle_can_internal_error(__unused canbus_error_data_t error_data) {
    safety_raise_fault(FAULT_CAN_INTERNAL_ERROR);
}

void safety_handle_can_receive_error(__unused enum canbus_receive_error_codes err_code) {
    //safety_raise_fault(FAULT_CAN_RECV_ERROR);
}

// ========================================
// Implementations for External Interface Functions
// ========================================

void safety_set_fault_led(bool on) {
    #ifdef MICRO_ROS_TRANSPORT_CAN
    canbus_set_device_in_error(on);
    #endif

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
    canbus_set_receive_error_cb(safety_handle_can_receive_error);
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