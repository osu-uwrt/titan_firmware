#include <assert.h>
#include "driver/canbus.h"
#include "driver/led.h"
#include "hardware/gpio.h"
#include "pico/binary_info.h"
#include "titan/logger.h"

#include <riptide_msgs2/msg/kill_switch_report.h>

#include "safety_interface.h"

#undef LOGGING_UNIT_NAME
#define LOGGING_UNIT_NAME "safety_interface"

// State values for software kill pin
#define SOFTKILL_STATE_KILL false
#define SOFTKILL_STATE_RUN  true

volatile bool safety_interface_kill_switch_refreshed = false;
volatile bool safety_interface_physical_kill_asserting_kill = true;
static volatile bool prev_kill_state = false;

static inline void safety_interface_refresh_physical_kill_switch(void) {
    // read the external switches
    bool kill_state = !gpio_get(PHYS_KILLSWITCH_PIN);
    safety_interface_physical_kill_asserting_kill = kill_state;
    safety_kill_switch_update(riptide_msgs2__msg__KillSwitchReport__KILL_SWITCH_PHYSICAL, kill_state, true);
}

static void safety_interface_gpio_callback(uint gpio, uint32_t events) {
    // Required since we're taking over the GPIO interrupt (and the SDK only supports 1 interrupt callback per core)
    can_mcp251xfd_interrupt_cb(gpio, events);
    if (gpio == PHYS_KILLSWITCH_PIN) {
        safety_interface_refresh_physical_kill_switch();
    }
}

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

    led_killswitch_set(false);
    gpio_put(SOFT_KILLSWITCH_PIN, SOFTKILL_STATE_KILL);

    if (!prev_kill_state) {
        prev_kill_state = true;
        safety_interface_kill_switch_refreshed = true;
    }
}

void safety_handle_enable(void) {
    led_killswitch_set(true);
    gpio_put(SOFT_KILLSWITCH_PIN, SOFTKILL_STATE_RUN);

    if (prev_kill_state) {
        prev_kill_state = false;
        safety_interface_kill_switch_refreshed = true;
    }
}

void safety_interface_setup(void) {
    // Initialize physical kill switch pin
    bi_decl_if_func_used(bi_1pin_with_name(PHYS_KILLSWITCH_PIN, "Kill Switch"));
    gpio_init(PHYS_KILLSWITCH_PIN);
    gpio_pull_up(PHYS_KILLSWITCH_PIN);
    gpio_set_dir(PHYS_KILLSWITCH_PIN, GPIO_IN);
    gpio_set_irq_enabled_with_callback(PHYS_KILLSWITCH_PIN, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true, &safety_interface_gpio_callback);

    // Initialize Physical Kill Override Pin
    // This allows the microcontroller to override the kill switch signal to kill the vehicle
    // Useful for software kill, so that we can kill power to the ESCs even if the kill switch is inserted
    bi_decl_if_func_used(bi_1pin_with_name(SOFT_KILLSWITCH_PIN, "Software Kill Control"));
    gpio_init(SOFT_KILLSWITCH_PIN);
    gpio_put(SOFT_KILLSWITCH_PIN, SOFTKILL_STATE_KILL); // Default to kill since safety is not initialized
    gpio_set_dir(SOFT_KILLSWITCH_PIN, GPIO_OUT);
    gpio_disable_pulls(SOFT_KILLSWITCH_PIN);

    canbus_set_internal_error_cb(safety_handle_can_internal_error);
}

void safety_interface_init(void) { }

void safety_interface_tick(void) {
    safety_interface_refresh_physical_kill_switch();
}

void safety_interface_deinit(void) { }


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
