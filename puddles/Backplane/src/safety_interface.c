#include "safety_interface.h"

#include "dshot.h"

#include "driver/led.h"
#include "pico/binary_info.h"
#include "pico/stdlib.h"

#include <riptide_msgs2/msg/kill_switch_report.h>

#include <assert.h>

bool safety_interface_kill_switch_refreshed = false;
static bool prev_kill_state = false;

static inline void safety_interface_refresh_physical_kill_switch(void) {
    // read the external switches
    bool kill_state = gpio_get(KILL_SW_SENSE);
    safety_kill_switch_update(riptide_msgs2__msg__KillSwitchReport__KILL_SWITCH_PHYSICAL, kill_state, true);
    dshot_notify_physical_kill_switch_change(kill_state);
}

static void safety_interface_gpio_callback(uint gpio, __unused uint32_t events) {
    if (gpio == KILL_SW_SENSE) {
        safety_interface_refresh_physical_kill_switch();
    }
}

// ========================================
// Implementations for External Interface Functions
// ========================================

void safety_set_fault_led(bool on) {
    led_fault_set(on);
}

void safety_handle_kill(void) {
    // Note: Any calls made in this function must be safe to be called from interrupts
    // This is because safety_kill_switch_update can be called from interrupts

    dshot_stop_thrusters();
    led_killswitch_set(false);

    if (!prev_kill_state) {
        prev_kill_state = true;
        safety_interface_kill_switch_refreshed = true;
    }
}

void safety_handle_enable(void) {
    led_killswitch_set(true);

    if (prev_kill_state) {
        prev_kill_state = false;
        safety_interface_kill_switch_refreshed = true;
    }
}

void safety_interface_setup(void) {
    // Initialize physical kill switch pin
    bi_decl_if_func_used(bi_1pin_with_name(KILL_SW_SENSE, "Kill Switch"));
    gpio_init(KILL_SW_SENSE);
    gpio_pull_up(KILL_SW_SENSE);
    gpio_set_dir(KILL_SW_SENSE, GPIO_IN);
    gpio_set_irq_enabled_with_callback(KILL_SW_SENSE, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true,
                                       &safety_interface_gpio_callback);
}

void safety_interface_init(void) {}

void safety_interface_tick(void) {
    safety_interface_refresh_physical_kill_switch();
}

void safety_interface_deinit(void) {}

// ========================================
// Constant Calculations
// Does not need to be modified - Edit safety_interface.h instead
// ========================================

// Define the kill switch variables used by core safety
// These are pulled from the riptide msg definitions
struct kill_switch_state kill_switch_states[riptide_msgs2__msg__KillSwitchReport__NUM_KILL_SWITCHES] = {
    [0 ... riptide_msgs2__msg__KillSwitchReport__NUM_KILL_SWITCHES - 1] = { .enabled = false }
};
const int num_kill_switches = sizeof(kill_switch_states) / sizeof(*kill_switch_states);
static_assert(sizeof(kill_switch_states) / sizeof(*kill_switch_states) <= 32, "Too many kill switches defined");

// Define the list of faults string so fault names can be easily looked up
#define DEFINE_STRING_NAME(name, id) [id] = #name,
const char *const fault_string_list[] = { XLIST_OF_FAULTS(DEFINE_STRING_NAME) };
const size_t fault_string_count = sizeof(fault_string_list) / sizeof(*fault_string_list);

const char *safety_lookup_fault_id(uint32_t fault_id) {
    const char *fault_name = NULL;
    if (fault_id < fault_string_count) {
        fault_name = fault_string_list[fault_id];
    }
    return (fault_name != NULL ? fault_name : "UNKNOWN");
}

// Verify we don't define any faults that are too big
#define DEFINE_FAULT_ID_STATIC_ASSERT(name, id)                                                                        \
    static_assert(id >= 0 && id <= MAX_FAULT_ID, "Invalid fault ID defined for " #name " (see below error)");
XLIST_OF_FAULTS(DEFINE_FAULT_ID_STATIC_ASSERT);
