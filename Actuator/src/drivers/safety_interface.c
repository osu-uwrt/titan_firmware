#include "drivers/safety.h"

#include "pico/binary_info.h"
#include "pico/stdlib.h"
#include "basic_logger/logging.h"

#include "actuators/claw.h"
#include "actuators/dropper.h"
#include "actuators/torpedo.h"

// ========================================
// Implementation dependent variable declarations
// ========================================

struct kill_switch_state kill_switch_states[NUM_KILL_SWITCHES] = 
    {[0 ... NUM_KILL_SWITCHES-1] = { .enabled = false }};

const int num_kill_switches = sizeof(kill_switch_states)/sizeof(*kill_switch_states);
static_assert(sizeof(kill_switch_states)/sizeof(*kill_switch_states) <= 32, "Too many kill switches defined");

// ========================================
// Implementations for External Interface Functions
// ========================================

static bool led_initialized = false;
void safety_set_fault_led(bool on) {
    bi_decl_if_func_used(bi_1pin_with_name(FAULT_LED_PIN, "Fault LED"));
    if (!led_initialized) {
        gpio_init(FAULT_LED_PIN);
        gpio_set_dir(FAULT_LED_PIN, GPIO_OUT);
        led_initialized = true;
    }
    gpio_put(FAULT_LED_PIN, on);
}

void safety_kill_robot(void) {
    dropper_safety_disable();
    claw_safety_disable();
    torpedo_safety_disable();
}

void safety_enable_robot(void) {
    
}

const char * safety_lookup_fault_id(uint32_t fault_id) {
    return (fault_id < sizeof(fault_string_list)/sizeof(*fault_string_list) ? fault_string_list[fault_id] : "UNKNOWN");
}