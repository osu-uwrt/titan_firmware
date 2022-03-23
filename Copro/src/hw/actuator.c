#include <stdio.h>
#include <stdlib.h>
#include "drivers/async_i2c.h"

#include "actuator_i2c/interface.h"
#include "basic_logging/logging.h"

#undef LOGGING_UNIT_NAME
#define LOGGING_UNIT_NAME "actuator_interface"

#define ACTUATOR_MAX_COMMANDS 8
#define ACTUATOR_I2C_BUS SENSOR_I2C_HW

struct actuator_command_data;
typedef void (*actuator_cmd_response_cb_t)(struct actuator_command_data*);
typedef struct actuator_command_data {
    actuator_i2c_cmd_t request;
    actuator_i2c_response_t response;
    struct async_i2c_request i2c_request;
    bool i2c_in_progress;
    actuator_cmd_response_cb_t response_cb;
    bool in_use;
} actuator_cmd_data_t;

static void actuator_command_done(__unused const struct async_i2c_request * req) {
    LOG_INFO("Request succesfully sent");
    actuator_cmd_data_t* cmd = (actuator_cmd_data_t*)req->user_data;
    uint8_t crc_calc = actuator_i2c_crc8_calc_response(&cmd->response, cmd->i2c_request.bytes_to_receive);
    if (crc_calc == cmd->response.crc8) {
        cmd->response_cb(cmd);
    } else {
        LOG_WARN("CRC Mismatch: 0x%02x calculated, 0x%02x received", crc_calc, cmd->response.crc8)
    }
    cmd->in_use = false;
}

static void actuator_command_failed(__unused const struct async_i2c_request * req, uint32_t abort_data){
    actuator_cmd_data_t* cmd = (actuator_cmd_data_t*)req->user_data;
    LOG_WARN("Failed to send actuator command %d: Abort Data 0x%x", cmd->request.cmd_id, abort_data);
    cmd->in_use = false;
}

// Allocate a static request for the start, so if can_allocate is false it doesn't have to wait for something to run when true
static actuator_cmd_data_t static_request;
static actuator_cmd_data_t *allocated_commands[ACTUATOR_MAX_COMMANDS] = {&static_request};
static int num_allocated_commands = 0;

/**
 * @brief Generates a command with the specified command id and response callback.
 * CAN RETURN NULL IF UNABLE TO GET A REQUEST
 * IT IS THE CALLERS RESPONSIBILITY TO HANDLE THIS
 * 
 * NOT INTERRUPT SAFE IF can_allocate is true
 * 
 * @param cmd_id The command id for the request
 * @param response_cb The callback to call on a successful response from the actuators
 * @param can_allocate If memory allocation is permitted during the running of this command. If memory cannot be allocated it relies on there being one request available
 * @return actuator_cmd_data_t* The command allocated or NULL if a command could not be generated
 */
static actuator_cmd_data_t* actuator_generate_command(enum actuator_command cmd_id, actuator_cmd_response_cb_t response_cb, bool can_allocate) {
    actuator_cmd_data_t * cmd = NULL;
    // Skip the static allocation to save for non-interrupt safe requests
    for (int i = (can_allocate ? 1 : 0); i < num_allocated_commands; i++) {
        if (!allocated_commands[i]->in_use) {
            allocated_commands[i]->in_use = true;
            cmd = allocated_commands[i];
        }
    }
    if (!cmd && num_allocated_commands < ACTUATOR_MAX_COMMANDS && can_allocate) {
        cmd = malloc(sizeof(*cmd));
        cmd->in_use = true;
        allocated_commands[num_allocated_commands++] = cmd;
    }

    if (cmd) {
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
    }

    return cmd;
}

static void actuator_send_command(actuator_cmd_data_t * cmd) {
    cmd->request.crc8 = actuator_i2c_crc8_calc_command(&cmd->request, cmd->i2c_request.bytes_to_send);

    async_i2c_enqueue(&cmd->i2c_request, &cmd->i2c_in_progress);
}




static void actuator_test_results(actuator_cmd_data_t * cmd) {
    LOG_INFO("Test Command Result: %d", cmd->response.data.result);
}

void actuator_test(void){
    actuator_cmd_data_t* cmd = actuator_generate_command(ACTUATOR_CMD_TEST, &actuator_test_results, true);
    if (!cmd) {
        LOG_ERROR("Failed to create actuator request");
        return;
    }
    cmd->request.data.test.data1 = 0xFEEDFACE;
    cmd->request.data.test.data2 = 0xDEADBEEF;
    actuator_send_command(cmd);
    
}