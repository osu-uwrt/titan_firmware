#include "actuators.h"

#include "actuators_internal.h"
#include "safety_interface.h"

#include "driver/dynamixel.h"
#include "pico/binary_info.h"
#include "pico/sync.h"
#include "titan/debug.h"
#include "titan/logger.h"

#include <string.h>

#undef LOGGING_UNIT_NAME
#define LOGGING_UNIT_NAME "actuators"

// Global exports
bool actuators_initialized = false;
volatile bool actuators_armed = false;

// Dynamixel Driver Configuration
#define countof(arr) (sizeof(arr) / sizeof(*(arr)))
#define MARKER_TORPEDO_ID 2
#define CLAW_ID 3
static const dynamixel_id dynamixel_servo_list[] = { MARKER_TORPEDO_ID, CLAW_ID };

// DXL Actuator Base State Storage
// Note these assume actuator IDs set above are sequential
#define DYNAMIXEL_FIRST_ID MARKER_TORPEDO_ID
static dxlact_state_t dynamixel_states[countof(dynamixel_servo_list)];

static dxlact_state_t *get_state_ptr(dynamixel_id id) {
    if (id < DYNAMIXEL_FIRST_ID)
        return NULL;
    id -= DYNAMIXEL_FIRST_ID;
    if (id >= countof(dynamixel_states))
        return NULL;
    return &dynamixel_states[id];
}

static dxlact_state_t *get_state_ptr_assert(dynamixel_id id) {
    dxlact_state_t *state = get_state_ptr(id);
    hard_assert(state != NULL);
    return state;
}

// ========================================
// Callbacks for Dynamixel Driver
// ========================================

static void actuators_dynamixel_error_cb(dynamixel_error_t error) {
    LOG_ERROR("Dynamixel Driver Error: %d (arg: %d) - %s line %d", error.fields.error, error.fields.wrapped_error_code,
              (error.fields.error_source == DYNAMIXEL_SOURCE_COMMS ? "dynamixel_comms" : "dynamixel_schedule"),
              error.fields.line);
    safety_raise_fault_with_arg(FAULT_ACTUATOR_FAILURE, error.data);
}

static void check_lower_actuator_unplugged_fault(void) {
    for (size_t i = 0; i < countof(dynamixel_servo_list); i++) {
        if (!dynamixel_check_connected(dynamixel_servo_list[i])) {
            return;
        }
    }
    safety_lower_fault(FAULT_ACTUATOR_UNPLUGGED);
}

static void actuators_dynamixel_event_cb(enum dynamixel_event event, dynamixel_id id) {
    dxlact_state_t *state = get_state_ptr(id);
    volatile struct dynamixel_ram *ram;

    switch (event) {
    case DYNAMIXEL_EVENT_CONNECTED:
        check_lower_actuator_unplugged_fault();
        if (state) {
            dxlact_base_report_connect(state);
        }
        break;

    case DYNAMIXEL_EVENT_DISCONNECTED:
        safety_raise_fault_with_arg(FAULT_ACTUATOR_UNPLUGGED, id);
        if (state) {
            dxlact_base_report_disconnect(state);
        }
        break;

    case DYNAMIXEL_EVENT_RAM_READ:
        if (state) {
            volatile struct dynamixel_ram *ram = dynamixel_get_ram(id);
            dxlact_base_report_state(state, ram->torque_enable, ram->moving, ram->goal_position, ram->present_position,
                                     ram->hardware_error_status);
        }
        break;

    case DYNAMIXEL_EVENT_ALERT:
        ram = dynamixel_get_ram(id);
        safety_raise_fault_with_arg(FAULT_ACTUATOR_HW_FAULT, (((uint32_t) id) << 24) | (ram->hardware_error_status));
        LOG_WARN("Dynamixel (ID: %d) reporting hardware error 0x%02x", id, ram->hardware_error_status);
        break;

    case DYNAMIXEL_EVENT_EEPROM_READ:
        // Don't care about eeprom reads, the status update will handle that
        break;
    }
}

// ========================================
// CANmore CLI Command Bindings
// ========================================

#define str_table_lookup(table, idx) ((idx) < (sizeof(table) / sizeof(*table)) ? table[idx] : "Unknown")
#define bool_lookup(val) ((val) ? "Yes" : "No")
static const char *const claw_states[] = { "ERROR",  "DISARMED", "UNKNOWN_POSITION", "OPENED",
                                           "CLOSED", "OPENING",  "CLOSING" };
static const char *const dropper_states[] = { "ERROR", "DISARMED", "READY", "DROPPING", "DROPPED" };
static const char *const torpedo_states[] = { "ERROR", "DISARMED", "CHARGING", "CHARGED", "FIRING", "FIRED" };

static int actuators_cmd_cb(size_t argc, const char *const *argv, FILE *fout) {
    if (argc < 2) {
        fprintf(fout, "Usage: @actuator [cmd]\n");
        return 2;
    }

    const char *cmd = argv[1];
    bool successful;
    const char *err = "No Error Occurred";

    if (!strcmp(cmd, "arm")) {
        successful = actuators_arm(&err);
    }
    else if (!strcmp(cmd, "disarm")) {
        actuators_disarm();
        successful = true;
    }
    else if (!strcmp(cmd, "status")) {
        fprintf(fout, "Actuators Busy:\t%s\n", bool_lookup(actuators_get_busy()));
        fprintf(fout, "Claw State:\t%s\n", str_table_lookup(claw_states, claw_get_state()));
        fprintf(fout, "Dropper State:\t\t%s\n", str_table_lookup(dropper_states, dropper_get_state()));
        fprintf(fout, "# Droppers Left:\t%d\n", dropper_get_available());
        fprintf(fout, "Torpedo State:\t\t%s\n", str_table_lookup(torpedo_states, torpedo_get_state()));
        fprintf(fout, "# Torpedoes Left:\t%d\n", torpedo_get_available());
        return 0;
    }
    else if (!strcmp(cmd, "drop")) {
        successful = dropper_drop_marker(&err);
    }
    else if (!strcmp(cmd, "fire")) {
        successful = torpedo_fire(&err);
    }
    else if (!strcmp(cmd, "notifyreload")) {
        successful = torpedo_notify_reload(&err);
    }
    else if (!strcmp(cmd, "gohome")) {
        successful = torpedo_marker_move_home(&err);
    }
    else if (!strcmp(cmd, "sethome")) {
        successful = torpedo_marker_set_home(&err);
    }
    else if (!strcmp(cmd, "clawopen")) {
        successful = claw_open(&err);
    }
    else if (!strcmp(cmd, "clawclose")) {
        successful = claw_close(&err);
    }
    else if (!strcmp(cmd, "setclosedpos")) {
        successful = claw_set_closed_position(&err);
    }
    else if (!strcmp(cmd, "clawmove")) {
        if (argc < 3) {
            fprintf(fout, "Usage: @actuator clawmove [delta]\n");
            return 2;
        }

        // Parse the provided delta value
        char *end;
        long long val = strtoll(argv[2], &end, 10);
        if (*end != 0 || end == argv[2]) {
            fprintf(fout, "Invalid number provided for delta\n");
            return 2;
        }
        if (val > INT32_MAX || val < INT32_MIN) {
            fprintf(fout, "Number provided for delta does not fit into int32\n");
            return 2;
        }

        successful = claw_creep_delta(&err, (int32_t) val);
    }
    else {
        fprintf(fout, "Unknown Command: '%s'\n", cmd);
        return 2;
    }

    // Report if successful or not
    if (!successful) {
        fprintf(fout, "Command '%s' Failed with Error: %s\n", cmd, err);
        return 1;
    }
    else {
        return 0;
    }
}

// ========================================
// Public Functions
// ========================================

void actuators_initialize(void) {
    hard_assert_if(ACTUATORS, actuators_initialized);
    bi_decl_if_func_used(bi_program_feature("Actuators V2"));

    bi_decl_if_func_used(bi_1pin_with_name(DYNAMIXEL_PWM_PIN, "Dynamixel TTL Signal"));
    gpio_disable_pulls(DYNAMIXEL_PWM_PIN);

    torpedo_marker_initialize(get_state_ptr_assert(MARKER_TORPEDO_ID), MARKER_TORPEDO_ID);
    claw_initialize(get_state_ptr_assert(CLAW_ID), CLAW_ID);

    dynamixel_init(pio0, 1, DYNAMIXEL_PWM_PIN, dynamixel_servo_list, countof(dynamixel_servo_list),
                   actuators_dynamixel_error_cb, actuators_dynamixel_event_cb);

    debug_remote_cmd_register("actuator", "[cmd]",
                              "Issues the requested actuator command\n"
                              "===Command Options===\n"
                              "Global Commands:\n"
                              "  arm:\t\tArms the actuator system\n"
                              "  disarm:\tDisarms the actuator system\n"
                              "  status:\tReports global actuator state\n"
                              "Torpedo/Marker Commands:\n"
                              "  drop:\t\tDrops a marker dropper\n"
                              "  fire:\t\tFires a torpedo\n"
                              "  notifyreload:\tNotifies that the dropper/torpedo has been reloaded\n"
                              "  gohome:\tSends the torpedo/marker to the home position\n"
                              "  sethome:\tSets the torpedo/marker to the home position to the current position\n"
                              "Claw Commands:\n"
                              "  clawopen:\tOpen the claw\n"
                              "  clawclose:\tClose the claw\n"
                              "  setclosedpos\tSets the claw closed position to the current position\n"
                              "  clawmove [d]:\tMoves the claw by [d] ticks",
                              actuators_cmd_cb);

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

    // Arm all of the subsystems
    for (size_t i = 0; i < countof(dynamixel_states); i++) {
        if (!dxlact_base_arm(&dynamixel_states[i], errMsgOut)) {
            // Failed, disarm all and report error
            actuators_disarm();
            return false;
        }
    }

    // Reset the torpedo/dropper count when we re-arm
    torpedo_marker_reset_count();

    return true;
}

void actuators_disarm(void) {
    actuators_armed = false;
    for (size_t i = 0; i < countof(dynamixel_states); i++) {
        dxlact_base_safety_disable(&dynamixel_states[i]);
    }
}

bool actuators_get_busy(void) {
    for (size_t i = 0; i < countof(dynamixel_states); i++) {
        if (dxlact_base_get_busy(&dynamixel_states[i])) {
            return true;
        }
    }
    return false;
}

void actuator_dxlitr_init(actuator_dxlitr_t *itr) {
    *itr = 0;
}

bool actuator_dxlitr_next(actuator_dxlitr_t *itr, riptide_msgs2__msg__DynamixelStatus *status_out) {
    volatile struct dynamixel_eeprom *eeprom = NULL;
    volatile struct dynamixel_ram *ram = NULL;

    // Iterate until match found or iterator reached the end
    while (*itr < countof(dynamixel_servo_list) && (eeprom == NULL || ram == NULL)) {
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
