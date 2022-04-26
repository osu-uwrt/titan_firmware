#include "drivers/safety.h"

#include "hw/actuator.h"
#include "hw/dio.h"
#include "hw/dshot.h"
#include "hw/esc_pwm.h"

#include <riptide_msgs2/msg/kill_switch_report.h>

// ========================================
// Implementation dependent variable declarations
// ========================================

struct kill_switch_state kill_switch_states[riptide_msgs2__msg__KillSwitchReport__NUM_KILL_SWITCHES] = 
    {[0 ... riptide_msgs2__msg__KillSwitchReport__NUM_KILL_SWITCHES-1] = { .enabled = false }};

const int num_kill_switches = sizeof(kill_switch_states)/sizeof(*kill_switch_states);
static_assert(sizeof(kill_switch_states)/sizeof(*kill_switch_states) <= 32, "Too many kill switches defined");

// ========================================
// Implementations for External Interface Functions
// ========================================

void safety_set_fault_led(bool on) {
    dio_set_fault_led(on);
}

void safety_kill_robot(void) {
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

void safety_enable_robot(void) {
    actuator_kill_report_refresh();
}

const char * safety_lookup_fault_id(uint32_t fault_id) {
    return (fault_id < sizeof(fault_string_list)/sizeof(*fault_string_list) ? fault_string_list[fault_id] : "UNKNOWN");
}