#include "drivers/safety.h"

// ========================================
// Implementation dependent variable declarations
// ========================================

struct kill_switch_state kill_switch_states[] = {};
const int num_kill_switches = 0;

// ========================================
// Implementations for External Interface Functions
// ========================================

void safety_set_fault_led(bool on) {
    gpio_put(FAULT_LED_PIN, on);
}

void safety_kill_robot(void) {

}

void safety_enable_robot(void) {

}

const char * safety_lookup_fault_id(uint32_t fault_id) {
    return (fault_id < sizeof(fault_string_list)/sizeof(*fault_string_list) ? fault_string_list[fault_id] : "UNKNOWN");
}