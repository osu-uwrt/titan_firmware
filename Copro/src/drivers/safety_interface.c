#include "drivers/safety.h"

#include "hw/actuator.h"
#include "hw/dio.h"
#include "hw/dshot.h"
#include "hw/esc_pwm.h"

#include <riptide_msgs2/msg/kill_switch_report.h>

#include "basic_logger/logging.h"

void safety_handle_can_internal_error(int data) {
    LOG_ERROR("CAN Internal Error: %d", data);
    safety_raise_fault(FAULT_CAN_INTERNAL_ERROR);
}

void safety_handle_can_receive_error(int data) {
    LOG_ERROR("CAN Receive Error: %d", data);
    safety_raise_fault(FAULT_CAN_RECV_ERROR);
}

// ========================================
// Implementations for External Interface Functions
// ========================================

void safety_set_fault_led(bool on) {
    dio_set_fault_led(on);
}

void safety_handle_kill(void) {
    // Note: Any calls made in this function must be safe to be called from interrupts
    // This is because safety_kill_switch_update can be called from interrupts

    #if HW_USE_DSHOT
    dshot_stop_thrusters();
    #endif
    #if HW_USE_PWM
    esc_pwm_stop_thrusters();
    #endif

    actuator_kill_report_refresh();
}

void safety_handle_enable(void) {
    actuator_kill_report_refresh();
}

void safety_interface_setup(void) {
    canbus_set_receive_error_cb(safety_handle_can_receive_error);
    canbus_set_internal_error_cb(safety_handle_can_internal_error);
}

void safety_interface_init(void) {

}

void safety_interface_tick(void) {

}


// ========================================
// Constant Calculations - Does not need to be modified
// ========================================

struct kill_switch_state kill_switch_states[riptide_msgs2__msg__KillSwitchReport__NUM_KILL_SWITCHES] =
    {[0 ... riptide_msgs2__msg__KillSwitchReport__NUM_KILL_SWITCHES-1] = { .enabled = false }};

const int num_kill_switches = sizeof(kill_switch_states)/sizeof(*kill_switch_states);
static_assert(sizeof(kill_switch_states)/sizeof(*kill_switch_states) <= 32, "Too many kill switches defined");

const char * safety_lookup_fault_id(uint32_t fault_id) {
    return (fault_id < sizeof(fault_string_list)/sizeof(*fault_string_list) ? fault_string_list[fault_id] : "UNKNOWN");
}