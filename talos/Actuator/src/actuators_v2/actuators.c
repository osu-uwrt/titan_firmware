#include "pico/binary_info.h"
#include "pico/sync.h"

#include "driver/dynamixel.h"
#include "titan/logger.h"

#include "actuators.h"
#include "actuators_internal.h"
#include "safety_interface.h"

#undef LOGGING_UNIT_NAME
#define LOGGING_UNIT_NAME "actuators"

// Global exports
bool actuators_initialized = false;
volatile bool actuators_armed = false;

#define MARKER_TORPEDO_ID 2
const dynamixel_id dynamixel_servo_list[] = {MARKER_TORPEDO_ID};
const size_t dynamixel_servo_count = sizeof(dynamixel_servo_list) / sizeof(*dynamixel_servo_list);

static void actuators_dynamixel_error_cb(dynamixel_error_t error) {
    LOG_ERROR("Dynamixel Driver Error: %d (arg: %d) - %s line %d", error.fields.error, error.fields.wrapped_error_code,
        (error.fields.error_source == DYNAMIXEL_SOURCE_COMMS ? "dynamixel_comms" : "dynamixel_schedule"), error.fields.line);
    safety_raise_fault_with_arg(FAULT_ACTUATOR_FAILURE, error);
}

static void check_lower_actuator_unplugged_fault(void) {
    for (size_t i = 0; i < dynamixel_servo_count; i++) {
        if (!dynamixel_check_connected(dynamixel_servo_list[i])) {
            return;
        }
    }
    safety_lower_fault(FAULT_ACTUATOR_UNPLUGGED);
}

static void actuators_dynamixel_event_cb(enum dynamixel_event event, dynamixel_id id) {
    volatile struct dynamixel_ram *ram;
    switch (event) {
        case DYNAMIXEL_EVENT_CONNECTED:
            check_lower_actuator_unplugged_fault();
            if (id == MARKER_TORPEDO_ID) {
                torpedo_marker_report_connect();
            }
            break;

        case DYNAMIXEL_EVENT_DISCONNECTED:
            safety_raise_fault(FAULT_ACTUATOR_UNPLUGGED);
            if (id == MARKER_TORPEDO_ID) {
                torpedo_marker_report_disconnect();
            }
            break;

        case DYNAMIXEL_EVENT_RAM_READ:
            ram = dynamixel_get_ram(id);
            if (id == MARKER_TORPEDO_ID) {
                torpedo_marker_report_state(ram->torque_enable, ram->moving, ram->goal_position,
                    ram->present_position, ram->hardware_error_status);
            }
            break;

        case DYNAMIXEL_EVENT_ALERT:
            ram = dynamixel_get_ram(id);
            safety_raise_fault_with_arg(FAULT_ACTUATOR_FAILURE, (((uint32_t)id) << 24) | (ram->hardware_error_status));
            LOG_WARN("Dynamixel (ID: %d) reporting hardware error 0x%02x", id, ram->hardware_error_status);
            break;

        case DYNAMIXEL_EVENT_EEPROM_READ:
            // Don't care about eeprom reads, the status update will handle that
            break;
    }
}

// Public Functions
void actuators_initialize(void) {
    hard_assert_if(ACTUATORS, actuators_initialized);
    bi_decl_if_func_used(bi_program_feature("Actuators V2"));

    bi_decl_if_func_used(bi_1pin_with_name(CLAW_CHECK_PIN, "Dynamixel TTL Signal"))
    gpio_disable_pulls(CLAW_CHECK_PIN);
    gpio_disable_pulls(CLAW_PWM_PIN);

    dynamixel_init(pio0, 1, CLAW_CHECK_PIN, dynamixel_servo_list, dynamixel_servo_count,
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

bool actuators_get_busy(void) {
    return torpedo_marker_get_busy();
}

void actuator_dxlitr_init(actuator_dxlitr_t *itr) {
    *itr = 0;
}

bool actuator_dxlitr_next(actuator_dxlitr_t *itr, riptide_msgs2__msg__DynamixelStatus *status_out) {
    volatile struct dynamixel_eeprom *eeprom = NULL;
    volatile struct dynamixel_ram *ram = NULL;

    // Iterate until match found or iterator reached the end
    while (*itr < dynamixel_servo_count && (eeprom == NULL || ram == NULL)) {
        eeprom = dynamixel_get_eeprom(dynamixel_servo_list[*itr]);
        ram = dynamixel_get_ram(dynamixel_servo_list[*itr]);
        (*itr)++;
    }

    // Check if iterator reached a valid point
    if (eeprom == NULL || ram == NULL) {
        return false;
    }

    // Populate status message
    status_out->model = eeprom->model_num;
    status_out->fw_version = eeprom->firmware_version;
    status_out->id = eeprom->id;
    status_out->hw_err_status = ram->hardware_error_status;
    status_out->torque_enable = ram->torque_enable;
    status_out->moving = ram->moving;
    status_out->voltage = ram->present_input_voltage;
    status_out->temperature = ram->present_temperature;
    status_out->position = ram->present_position;
    status_out->load = ram->present_load;
    return true;
}
