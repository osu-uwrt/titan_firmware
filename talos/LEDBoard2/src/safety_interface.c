#include "safety_interface.h"

#include "driver/led.h"
#include "titan/logger.h"

#include <assert.h>

#undef LOGGING_UNIT_NAME
#define LOGGING_UNIT_NAME "safety_interface"

#ifdef MICRO_ROS_TRANSPORT_CAN
#include "driver/canbus.h"

static void safety_handle_can_internal_error(canbus_error_data_t error_data) {
    LOG_ERROR("CAN Internal Error - Line: %d; Code: %d (%s Error)", error_data.error_line, error_data.error_code,
              (error_data.is_driver_error ? "Internal Driver" : "Library"));
    safety_raise_fault_with_arg(FAULT_CAN_INTERNAL_ERROR, error_data.raw);
}

#endif

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

    // TODO: Modify this function to add callbacks when system is killed
    led_killswitch_set(false);
}

void safety_handle_enable(void) {
    // TODO: Modify this function to add callbacks for when system is enabled

    led_killswitch_set(true);
}

void safety_interface_setup(void) {
#ifdef MICRO_ROS_TRANSPORT_CAN
    canbus_set_internal_error_cb(safety_handle_can_internal_error);
#endif
}

void safety_interface_init(void) {
    // TODO: Modify this function to add code to be called during safety_init
}

void safety_interface_tick(void) {}

void safety_interface_deinit(void) {
    // TODO: Modify this function to add code to be called during safety_deinit
}

// ========================================
// Constant Calculations
// Does not need to be modified - Edit safety_interface.h instead
// ========================================

// Define the kill switch variables used by core safety
struct kill_switch_state kill_switch_states[NUM_KILL_SWITCHES];
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
