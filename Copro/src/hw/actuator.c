#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "drivers/async_i2c.h"

#include "actuator_i2c/interface.h"
#include "basic_logging/logging.h"

#include "drivers/safety.h"

#undef LOGGING_UNIT_NAME
#define LOGGING_UNIT_NAME "actuator_interface"

#define ACTUATOR_MAX_COMMANDS 8
#define ACTUATOR_I2C_BUS SENSOR_I2C_HW

#define ACTUATOR_POLLING_RATE_MS 300
#define ACTUATOR_MAX_STATUS_AGE_MS 1000

// ========================================
// I2C Command Generation/Processing
// ========================================

struct actuator_command_data;
typedef void (*actuator_cmd_response_cb_t)(struct actuator_command_data*);
typedef struct actuator_command_data {
    actuator_i2c_cmd_t request;
    actuator_i2c_response_t response;
    struct async_i2c_request i2c_request;
    bool i2c_in_progress;
    actuator_cmd_response_cb_t response_cb;
    bool important_request;     // Set to true if the command being lost should be a fault
    bool in_use;
} actuator_cmd_data_t;

static void actuator_command_done(__unused const struct async_i2c_request * req) {
    actuator_cmd_data_t* cmd = (actuator_cmd_data_t*)req->user_data;

    if (cmd->i2c_request.bytes_to_receive > 0) {
        uint8_t crc_calc = actuator_i2c_crc8_calc_response(&cmd->response, cmd->i2c_request.bytes_to_receive);
        if (crc_calc == cmd->response.crc8) {
            if (cmd->response_cb){
                cmd->response_cb(cmd);
            }
        } else {
            LOG_WARN("CRC Mismatch: 0x%02x calculated, 0x%02x received", crc_calc, cmd->response.crc8)
        }
    } else {
        if (cmd->response_cb){
            cmd->response_cb(cmd);
        }
    }
    cmd->in_use = false;
}

static void actuator_command_failed(__unused const struct async_i2c_request * req, uint32_t abort_data){
    actuator_cmd_data_t* cmd = (actuator_cmd_data_t*)req->user_data;
    if (cmd->important_request) {
        LOG_ERROR("Failed to send important actuator command %d: Abort Data 0x%x", cmd->request.cmd_id, abort_data);
        safety_raise_fault(FAULT_ACTUATOR_FAIL);
    } else {
        LOG_WARN("Failed to send actuator command %d: Abort Data 0x%x", cmd->request.cmd_id, abort_data);
    }
    cmd->in_use = false;
}

/**
 * @brief Populates cmd with the required elements for an actuator i2c command
 * 
 * @param cmd The command id for the request
 * @param cmd_id The callback to call on a successful response from the actuators. Can be NULL if no response is needed
 * @param response_cb The callback to call on a successful response from the actuators. Can be NULL if no response is needed
 * @param important If a failure of this command should be made immediately known
 */
static void actuator_populate_command(actuator_cmd_data_t* cmd, enum actuator_command cmd_id, actuator_cmd_response_cb_t response_cb, bool important) {
    cmd->i2c_request.i2c = ACTUATOR_I2C_BUS;
    cmd->i2c_request.address = ACTUATOR_I2C_ADDR;
    cmd->i2c_request.nostop = false;
    cmd->i2c_request.tx_buffer = (uint8_t*)(&cmd->request);
    cmd->i2c_request.rx_buffer = (uint8_t*)(&cmd->response);
    cmd->i2c_request.bytes_to_send = ACTUATOR_GET_CMD_SIZE(cmd_id);
    cmd->i2c_request.bytes_to_receive = ACTUATOR_GET_RESPONSE_SIZE(cmd_id);
    cmd->i2c_request.completed_callback = actuator_command_done;
    cmd->i2c_request.failed_callback = actuator_command_failed;
    cmd->i2c_request.next_req_on_success = NULL;
    cmd->i2c_request.user_data = cmd;

    cmd->i2c_in_progress = false;
    cmd->response_cb = response_cb;

    cmd->request.cmd_id = cmd_id;
    cmd->important_request = important;
}


static actuator_cmd_data_t *allocated_commands[ACTUATOR_MAX_COMMANDS];
static int num_allocated_commands = 0;
/**
 * @brief Generates a command with the specified command id and response callback.
 * CAN RETURN NULL IF UNABLE TO GET A REQUEST
 * IT IS THE CALLERS RESPONSIBILITY TO HANDLE THIS
 * 
 * NOT INTERRUPT SAFE
 * 
 * @param cmd_id The command id for the request
 * @param response_cb The callback to call on a successful response from the actuators. Can be NULL if no response is needed
 * @return actuator_cmd_data_t* The command allocated or NULL if a command could not be generated
 */
static actuator_cmd_data_t* actuator_generate_command(enum actuator_command cmd_id, actuator_cmd_response_cb_t response_cb) {
    actuator_cmd_data_t * cmd = NULL;
    // Skip the static allocation to save for non-interrupt safe requests
    for (int i = 0; i < num_allocated_commands; i++) {
        if (!allocated_commands[i]->in_use) {
            allocated_commands[i]->in_use = true;
            cmd = allocated_commands[i];
        }
    }
    if (!cmd && num_allocated_commands < ACTUATOR_MAX_COMMANDS) {
        cmd = malloc(sizeof(*cmd));
        cmd->in_use = true;
        allocated_commands[num_allocated_commands++] = cmd;
    }

    if (cmd) {
        actuator_populate_command(cmd, cmd_id, response_cb, true);
    }

    return cmd;
}

static void actuator_send_command(actuator_cmd_data_t * cmd) {
    cmd->request.crc8 = actuator_i2c_crc8_calc_command(&cmd->request, cmd->i2c_request.bytes_to_send);

    async_i2c_enqueue(&cmd->i2c_request, &cmd->i2c_in_progress);
}

// ========================================
// Actuator Board Monitoring
// ========================================

struct actuator_i2c_status actuator_last_status;

static actuator_cmd_data_t status_command = {.in_use = false, .i2c_in_progress = false};
static actuator_cmd_data_t kill_switch_update_command = {.in_use = false, .i2c_in_progress = false};
static absolute_time_t status_valid_timeout = {0};
static bool version_warning_printed = false;

static void actuator_kill_switch_update_callback(actuator_cmd_data_t * cmd) {
    if (cmd->response.data.result != ACTUATOR_RESULT_SUCCESSFUL) {
        LOG_ERROR("Non-successful kill switch update response %d", cmd->response.data.result);
        safety_raise_fault(FAULT_ACTUATOR_FAIL);
    }
}

static void actuator_status_callback(actuator_cmd_data_t * cmd) {
    struct actuator_i2c_status *status = &cmd->response.data.status;
    if (status->firmware_status.version_major != ACTUATOR_EXPECTED_FIRMWARE_MAJOR && status->firmware_status.version_major != ACTUATOR_EXPECTED_FIRMWARE_MINOR) {
        if (!version_warning_printed) {
            LOG_ERROR("Invalid firmware version found: %d.%d (%d.%d expected)", status->firmware_status.version_major, status->firmware_status.version_minor, ACTUATOR_EXPECTED_FIRMWARE_MAJOR, ACTUATOR_EXPECTED_FIRMWARE_MINOR);
            safety_raise_fault(FAULT_ACTUATOR_FAIL);
            version_warning_printed = true;
        }
    } else {
        memcpy(&actuator_last_status, status, sizeof(*status));
        status_valid_timeout = make_timeout_time_ms(ACTUATOR_MAX_STATUS_AGE_MS);

        if (safety_initialized) {
            kill_switch_update_command.request.data.kill_switch.asserting_kill = safety_kill_get_asserting_kill();
            kill_switch_update_command.in_use = true;
            actuator_send_command(&kill_switch_update_command);
        }
    }
}

/**
 * @brief Alarm callback to poll the actuator board
 * 
 * @param id The ID of the alarm that triggered the callback
 * @param user_data User provided data. This is NULL
 * @return int64_t If/How to restart the timer
 */
static int64_t actuator_poll_alarm_callback(__unused alarm_id_t id, __unused void *user_data) {
    if (status_command.in_use) {
        LOG_ERROR("Unable to poll actuator board, request still in progress");
        safety_raise_fault(FAULT_ACTUATOR_FAIL);
    } else {
        // TODO: Raise fault if actuator not connected
        status_command.in_use = true;
        actuator_send_command(&status_command);
    }

    return ACTUATOR_POLLING_RATE_MS * 1000;
}

// ========================================
// Public Command Requests
// ========================================

static void actuator_generic_result_cb(actuator_cmd_data_t * cmd) {
    if (cmd->response.data.result == ACTUATOR_RESULT_FAILED){
        if (cmd->important_request) {
            LOG_ERROR("Request %d returned failed result", cmd->response.data.result);
            safety_raise_fault(FAULT_ACTUATOR_FAIL);
        } else {
            LOG_WARN("Non-critical request %d returned failed result", cmd->response.data.result);
        }
    }
}

void actuator_open_claw(void) {
    actuator_cmd_data_t* cmd = actuator_generate_command(ACTUATOR_CMD_OPEN_CLAW, actuator_generic_result_cb);
    if (!cmd) {
        LOG_ERROR("Failed to create request");
        safety_raise_fault(FAULT_ACTUATOR_FAIL);
        return;
    }
    actuator_send_command(cmd);
}

void actuator_close_claw(void) {
    actuator_cmd_data_t* cmd = actuator_generate_command(ACTUATOR_CMD_CLOSE_CLAW, actuator_generic_result_cb);
    if (!cmd) {
        LOG_ERROR("Failed to create request");
        safety_raise_fault(FAULT_ACTUATOR_FAIL);
        return;
    }
    actuator_send_command(cmd);
}

void actuator_set_claw_timings(uint16_t open_time_ms, uint16_t close_time_ms) {
    actuator_cmd_data_t* cmd = actuator_generate_command(ACTUATOR_CMD_CLAW_TIMING, actuator_generic_result_cb);
    if (!cmd) {
        LOG_ERROR("Failed to create request");
        safety_raise_fault(FAULT_ACTUATOR_FAIL);
        return;
    }
    cmd->request.data.claw_timing.open_time_ms = open_time_ms;
    cmd->request.data.claw_timing.close_time_ms = close_time_ms;
    actuator_send_command(cmd);
}

void actuator_arm_torpedo(void) {
    actuator_cmd_data_t* cmd = actuator_generate_command(ACTUATOR_CMD_ARM_TORPEDO, actuator_generic_result_cb);
    if (!cmd) {
        LOG_ERROR("Failed to create request");
        safety_raise_fault(FAULT_ACTUATOR_FAIL);
        return;
    }
    actuator_send_command(cmd);
}

void actuator_disarm_torpedo(void) {
    actuator_cmd_data_t* cmd = actuator_generate_command(ACTUATOR_CMD_DISARM_TORPEDO, actuator_generic_result_cb);
    if (!cmd) {
        LOG_ERROR("Failed to create request");
        safety_raise_fault(FAULT_ACTUATOR_FAIL);
        return;
    }
    actuator_send_command(cmd);
}

void actuator_fire_torpedo(uint8_t torpedo_id) {
    actuator_cmd_data_t* cmd = actuator_generate_command(ACTUATOR_CMD_FIRE_TORPEDO, actuator_generic_result_cb);
    if (!cmd) {
        LOG_ERROR("Failed to create request");
        safety_raise_fault(FAULT_ACTUATOR_FAIL);
        return;
    }
    cmd->request.data.fire_torpedo.torpedo_num = torpedo_id;
    actuator_send_command(cmd);
}

void actuator_set_torpedo_timings(uint8_t torpedo_id, enum torpedo_timing_type timing_type, uint16_t time_us) {
    actuator_cmd_data_t* cmd = actuator_generate_command(ACTUATOR_CMD_TORPEDO_TIMING, actuator_generic_result_cb);
    if (!cmd) {
        LOG_ERROR("Failed to create request");
        safety_raise_fault(FAULT_ACTUATOR_FAIL);
        return;
    }
    cmd->request.data.torpedo_timing.torpedo_num = torpedo_id;
    cmd->request.data.torpedo_timing.timing_type = timing_type;
    cmd->request.data.torpedo_timing.time_us = time_us;
    actuator_send_command(cmd);
}

void actuator_drop_marker(uint8_t marker_id) {
    actuator_cmd_data_t* cmd = actuator_generate_command(ACTUATOR_CMD_DROP_MARKER, actuator_generic_result_cb);
    if (!cmd) {
        LOG_ERROR("Failed to create request");
        safety_raise_fault(FAULT_ACTUATOR_FAIL);
        return;
    }
    cmd->request.data.drop_marker.marker_num = marker_id;
    actuator_send_command(cmd);
}

void actuator_clear_dropper_status(void) {
    actuator_cmd_data_t* cmd = actuator_generate_command(ACTUATOR_CMD_CLEAR_DROPPER_STATUS, actuator_generic_result_cb);
    if (!cmd) {
        LOG_ERROR("Failed to create request");
        safety_raise_fault(FAULT_ACTUATOR_FAIL);
        return;
    }
    actuator_send_command(cmd);
}

void actuator_set_marker_timings(uint16_t active_time_ms) {
    actuator_cmd_data_t* cmd = actuator_generate_command(ACTUATOR_CMD_MARKER_TIMING, actuator_generic_result_cb);
    if (!cmd) {
        LOG_ERROR("Failed to create request");
        safety_raise_fault(FAULT_ACTUATOR_FAIL);
        return;
    }
    cmd->request.data.marker_timing.active_time_ms = active_time_ms;
    actuator_send_command(cmd);
}

void actuator_reset_actuators(void) {
    actuator_cmd_data_t* cmd = actuator_generate_command(ACTUATOR_CMD_RESET_ACTUATORS, NULL);
    if (!cmd) {
        LOG_ERROR("Failed to create request");
        safety_raise_fault(FAULT_ACTUATOR_FAIL);
        return;
    }
    actuator_send_command(cmd);
}

// ========================================
// Misc Public Methods
// ========================================

bool actuator_initialized = false;

bool actuator_is_connected(void) {
    if (actuator_initialized) {
        return absolute_time_diff_us(status_valid_timeout, get_absolute_time()) < 0;
    } else {
        return false;
    }
}

void actuator_init(void){
    hard_assert_if(LIFETIME_CHECK, actuator_initialized);

    status_valid_timeout = get_absolute_time(); // Expire the last status immediately
    actuator_initialized = true;
    actuator_populate_command(&status_command, ACTUATOR_CMD_GET_STATUS, actuator_status_callback, false);
    actuator_populate_command(&kill_switch_update_command, ACTUATOR_CMD_KILL_SWITCH, actuator_kill_switch_update_callback, true);
    hard_assert(add_alarm_in_ms(ACTUATOR_POLLING_RATE_MS, &actuator_poll_alarm_callback, NULL, true) > 0);
}
