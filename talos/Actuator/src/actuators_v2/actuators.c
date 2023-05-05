#include "pico/binary_info.h"
#include "pico/sync.h"
#include "actuators.h"
#include "actuators_internal.h"
#include "safety_interface.h"
#include "basic_logger/logging.h"
#include "dynamixel/dynamixel.h"

#undef LOGGING_UNIT_NAME
#define LOGGING_UNIT_NAME "actuators"

// Global exports
bool actuators_initialized = false;
volatile bool actuators_armed = false;

#define MARKER_TORPEDO_ID 2
const dynamixel_id dynamixel_servo_list[] = {MARKER_TORPEDO_ID};
const size_t dynamixel_servo_count = sizeof(dynamixel_servo_list) / sizeof(*dynamixel_servo_list);

void actuators_dynamixel_error_cb(dynamixel_error_t error) {
    LOG_ERROR("Dynamixel Driver Error: %d (arg: %d) - %s line %d", error.fields.error, error.fields.wrapped_error_code,
        (error.fields.error_source == DYNAMIXEL_SOURCE_COMMS ? "dynamixel_comms" : "dynamixel_schedule"), error.fields.line);
    safety_raise_fault(FAULT_ACTUATOR_FAILURE);
}

static void check_lower_actuator_unplugged_fault(void) {
    for (size_t i = 0; i < dynamixel_servo_count; i++) {
        if (!dynamixel_check_connected(dynamixel_servo_list[i])) {
            return;
        }
    }
    safety_lower_fault(FAULT_ACTUATOR_UNPLUGGED);
}

void actuators_dynamixel_event_cb(enum dynamixel_event event, dynamixel_id id) {
    switch (event) {
        case DYNAMIXEL_EVENT_CONNECTED:
            check_lower_actuator_unplugged_fault();
            // TODO: Notify drivers that the actuator is back
            break;

        case DYNAMIXEL_EVENT_DISCONNECTED:
            safety_raise_fault(FAULT_ACTUATOR_UNPLUGGED);
            // TODO: Check if any other things need to be cleared
            break;

        case DYNAMIXEL_EVENT_RAM_READ:
            break;

        case DYNAMIXEL_EVENT_ALERT:
            break;
    }
}

// Public Functions
void actuators_initialize(void) {
    hard_assert_if(ACTUATORS, actuators_initialized);
    bi_decl_if_func_used(bi_program_feature("Actuators V2"));

    dynamixel_init(pio0, 0, CLAW_CHECK_PIN, dynamixel_servo_list, dynamixel_servo_count,
                    actuators_dynamixel_error_cb, actuators_dynamixel_event_cb);

    torpedo_marker_initialize(MARKER_TORPEDO_ID);

    actuators_initialized = true;
}

bool actuators_arm(const char **errMsgOut) {
    hard_assert_if(ACTUATORS, !actuators_initialized);

    // Make sure a kill switch interrupt won't fire in between checking conditions and arming
    uint32_t prev_interrupts = save_and_disable_interrupts();

    // Don't allow arming if already armed
    if (actuators_armed) {
        restore_interrupts(prev_interrupts);
        *errMsgOut = "Already Armed";
        return false;
    }

    // Don't allow arming if killed
    if (safety_kill_get_asserting_kill()) {
        restore_interrupts(prev_interrupts);
        *errMsgOut = "Kill Switch Removed";
        return false;
    }

    // We're good to arm
    actuators_armed = true;
    restore_interrupts(prev_interrupts);

    LOG_INFO("Arming Actuators");

    return torpedo_marker_arm(errMsgOut);
}

void actuators_disarm(void) {
    actuators_armed = false;
    torpedo_marker_safety_disable();
}
