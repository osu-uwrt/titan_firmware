#ifndef _ASYNC_I2C_H
#define _ASYNC_I2C_H

#include <stdint.h>
#include <stdbool.h>

#include "hardware/i2c.h"
#include "actuator_i2c/interface.h"

// PICO_CONFIG: PARAM_ASSERTIONS_ENABLED_ASYNC_I2C, Enable/disable assertions in the Async I2C Target module, type=bool, default=0, group=Actuator
#ifndef PARAM_ASSERTIONS_ENABLED_ASYNC_I2C_TARGET
#define PARAM_ASSERTIONS_ENABLED_ASYNC_I2C_TARGET 0
#endif

// I2C Request Types
struct async_i2c_request;

/**
 * @brief Typedef for callback to i2c function
 */
typedef void (*async_i2c_cb_t)(const struct async_i2c_request *);
typedef void (*async_i2c_abort_cb_t)(const struct async_i2c_request *, uint32_t);
struct async_i2c_request {
    i2c_inst_t *i2c;
    uint8_t address;
    bool nostop;
    const uint8_t *tx_buffer;
    uint8_t *rx_buffer;
    uint16_t bytes_to_send;
    uint16_t bytes_to_receive;
    async_i2c_cb_t completed_callback;
    async_i2c_abort_cb_t failed_callback;
    const struct async_i2c_request *next_req_on_success;
    void* user_data;
};

/**
 * @brief Bool if async_i2c_target_init has been called
 */
extern bool async_i2c_target_initialized;

/**
 * @brief Attempts to get the next command from the I2C bus.
 * If there is a command received it is written into command.
 * If no command is received then it will return false.
 * 
 * @param cmd The output for the received command
 * @return true Succesfully received a command into cmd
 * @return false No command has been received
 */
bool async_i2c_target_get_next_command(actuator_i2c_cmd_t *cmd);

/**
 * @brief Finishes the active command and reponds with the given response.
 * If response is NULL then there will be no response sent with the command.
 *
 * This function handles crc8 calculation for the response
 * 
 * @param response Nullable I2C response
 * @param size The size to send with the response
 */
void async_i2c_target_finish_command(actuator_i2c_response_t *response, size_t size);

/**
 * @brief Initialize async i2c and the corresponding i2c hardware in target mode
 * 
 * @param baudrate The data rate of the i2c bus in Hz
 * @param i2c_address The address for the i2c device to use
 */
void async_i2c_target_init(uint baudrate, uint8_t i2c_address);

#endif