#include "safety.h"

#include "pico/stdlib.h"
#include "basic_logger/logging.h"

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
#define FAULT_LED_PIN BUILTIN_LED3_PIN
void safety_set_fault_led(bool on) {
    if (!led_initialized) {
        gpio_init(FAULT_LED_PIN);
        gpio_set_dir(FAULT_LED_PIN, GPIO_OUT);
        led_initialized = true;
    }
    gpio_put(FAULT_LED_PIN, on);
}

void safety_kill_robot(void) {
    // TODO: Implement kill robot
    LOG_INFO("Disabling Robot");
}

void safety_enable_robot(void) {
    LOG_INFO("Enbling Robot");
}

const char * safety_lookup_fault_id(uint32_t fault_id) {
    return (fault_id < sizeof(fault_string_list)/sizeof(*fault_string_list) ? fault_string_list[fault_id] : "UNKNOWN");
}