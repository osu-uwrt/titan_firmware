#ifndef _DEPTH_SENSOR_COMMANDS_H
#define _DEPTH_SENSOR_COMMANDS_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "async_i2c.h"
#include "safety_interface.h"

// Dirty hack to allow commands to be in separate header file
static void depth_init_failure(const struct async_i2c_request *req, uint32_t abort_data);
static void depth_read_failure(const struct async_i2c_request *req, uint32_t abort_data);

static void depth_reset_finished(const struct async_i2c_request *req);
static void depth_prom_read_finished(const struct async_i2c_request *req);
static void depth_convert_cmd_finished(const struct async_i2c_request *req);
static void depth_adc_read_finished(const struct async_i2c_request *req);

#define DEPTH_I2C_NUM SENSOR_I2C
#define DEPTH_I2C_ADDR 0x76

#define DEPTH_CMD_RESET 0x1E
#define DEPTH_CMD_CONVERT_D1(oversampling) (0x40+(oversampling << 1))
#define DEPTH_CMD_CONVERT_D2(oversampling) (0x50+(oversampling << 1))
#define DEPTH_CMD_ADC_READ 0x00
#define DEPTH_CMD_PROM_READ(addr) (0xA0 + (addr << 1))

#define DEPTH_PROM_EXPECTED_ID 0b0000001101000000
#define DEPTH_PROM_ID_MASK     0b0000111111100000

#define DEPTH_OVERSAMPLING 5

static const uint8_t reset_cmd[] = {DEPTH_CMD_RESET};
static const struct async_i2c_request reset_req = {
    .i2c_num = DEPTH_I2C_NUM,
    .address = DEPTH_I2C_ADDR,
    .nostop = false,
    .tx_buffer = reset_cmd,
    .rx_buffer = NULL,
    .bytes_to_send = sizeof(reset_cmd),
    .bytes_to_receive = 0,
    .completed_callback = &depth_reset_finished,
    .failed_callback = &depth_init_failure,
    .next_req_on_success = NULL
};

// Note all prom reads will use this command
// The prom_read_cmd is not const so it can be written to and the request struct reused
static uint8_t prom_read_cmd[] = {DEPTH_CMD_PROM_READ(0)};
static uint8_t prom_read_data[2];
static const struct async_i2c_request prom_read_req = {
    .i2c_num = DEPTH_I2C_NUM,
    .address = DEPTH_I2C_ADDR,
    .nostop = false,
    .tx_buffer = prom_read_cmd,
    .rx_buffer = prom_read_data,
    .bytes_to_send = sizeof(prom_read_cmd),
    .bytes_to_receive = sizeof(prom_read_data),
    .completed_callback = &depth_prom_read_finished,
    .failed_callback = &depth_init_failure,
    .next_req_on_success = NULL
};


static const uint8_t d1_convert_cmd[] = {DEPTH_CMD_CONVERT_D1(DEPTH_OVERSAMPLING)};
static const struct async_i2c_request d1_convert_req = {
    .i2c_num = DEPTH_I2C_NUM,
    .address = DEPTH_I2C_ADDR,
    .nostop = false,
    .tx_buffer = d1_convert_cmd,
    .rx_buffer = NULL,
    .bytes_to_send = sizeof(d1_convert_cmd),
    .bytes_to_receive = 0,
    .completed_callback = &depth_convert_cmd_finished,
    .failed_callback = &depth_read_failure,
    .next_req_on_success = NULL
};

static const uint8_t d2_convert_cmd[] = {DEPTH_CMD_CONVERT_D2(DEPTH_OVERSAMPLING)};
static const struct async_i2c_request d2_convert_req = {
    .i2c_num = DEPTH_I2C_NUM,
    .address = DEPTH_I2C_ADDR,
    .nostop = false,
    .tx_buffer = d2_convert_cmd,
    .rx_buffer = NULL,
    .bytes_to_send = sizeof(d2_convert_cmd),
    .bytes_to_receive = 0,
    .completed_callback = &depth_convert_cmd_finished,
    .failed_callback = &depth_read_failure,
    .next_req_on_success = NULL
};

static const uint8_t adc_read_cmd[] = {DEPTH_CMD_ADC_READ};
static uint8_t adc_read_data[3];
static const struct async_i2c_request adc_read_req = {
    .i2c_num = DEPTH_I2C_NUM,
    .address = DEPTH_I2C_ADDR,
    .nostop = false,
    .tx_buffer = adc_read_cmd,
    .rx_buffer = adc_read_data,
    .bytes_to_send = sizeof(adc_read_cmd),
    .bytes_to_receive = sizeof(adc_read_data),
    .completed_callback = &depth_adc_read_finished,
    .failed_callback = &depth_read_failure,
    .next_req_on_success = NULL
};

#endif