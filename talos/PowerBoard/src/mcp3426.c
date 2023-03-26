#include "mcp3426.h"
#include "pico/stdlib.h"
#include "stdint.h"
#include "basic_logger/logging.h"
#include "safety_interface.h"
#include "async_i2c.h"
#include "stdio.h"

#undef LOGGING_UNIT_NAME
#define LOGGING_UNIT_NAME "mcp3426_adc"

#define MCP3426_ADDR 0x68 // 1101000
#define MCP3426_GENERAL_CALL_RESET 0x06

// Modes used internally by the driver
#define MCP3426_MODE_UNINITIALIZED 2
#define MCP3426_MODE_INITIALIZING 2

#define MCP3426_START_READ 1
#define MCP3426_WAIT_TO_READ 0

#define I2C_NUM BOARD_I2C

/// Creates a MCP3426 configuration byte.
#define mcp3426_config_reg(read, chan, mode, rate, gain) (read << 7) | (chan << 5) | (mode << 4) | (rate << 2) | (gain)

static void mcp3426_on_read(const struct async_i2c_request *req);
static void mcp3426_on_write(const struct async_i2c_request *req);
static void mcp3426_on_setup_complete(const struct async_i2c_request *req);

static void mcp3426_on_error(const struct async_i2c_request *req, uint32_t error_code) {
    (void) req;

    LOG_WARN("ADC failure: ", error_code);
    safety_raise_fault(FAULT_ADC_ERROR);
}

static uint8_t set_channel_cmds[MCP3426_CHANNEL_COUNT] = {0};
static struct async_i2c_request set_channel_req = {
    .i2c_num = I2C_NUM,
    .address = MCP3426_ADDR,
    .nostop = false,
    .tx_buffer = NULL,
    .rx_buffer = NULL,
    .bytes_to_send = 0,
    .bytes_to_receive = 0,
    .completed_callback = &mcp3426_on_write,
    .failed_callback = &mcp3426_on_error,
    .next_req_on_success = NULL,
};

static uint8_t general_call_reset_cmd = 0x06;
static struct async_i2c_request general_call_reset_req = {
    .i2c_num = I2C_NUM,
    .address = 0x00,
    .nostop = false,
    .tx_buffer = &general_call_reset_cmd,
    .rx_buffer = NULL,
    .bytes_to_send = 1,
    .bytes_to_receive = 0,
    .completed_callback = &mcp3426_on_setup_complete,
    .failed_callback = &mcp3426_on_error, // TODO
    .next_req_on_success = NULL,
};

static uint8_t rx_buffer[3];
static struct async_i2c_request read_req = {
    .i2c_num = I2C_NUM,
    .address = MCP3426_ADDR,
    .nostop = false,
    .tx_buffer = NULL,
    .rx_buffer = rx_buffer,
    .bytes_to_send = 0,
    .bytes_to_receive = 3,
    .completed_callback = mcp3426_on_read,
    .failed_callback = &mcp3426_on_error,
    .next_req_on_success = NULL,
};
static int mode = MCP3426_MODE_UNINITIALIZED;

static bool msg_in_progress = false;

static float adc_values[MCP3426_CHANNEL_COUNT] = {0};

static int64_t mcp3426_enqueue_read(__unused alarm_id_t id, __unused void *user_data) {
    async_i2c_enqueue(&read_req, &msg_in_progress);

    return 0;
}

static void mcp3426_on_write(const struct async_i2c_request *req) {
    (void) req;
    add_alarm_in_ms(100, &mcp3426_enqueue_read, NULL, true);
}

static void mcp3426_set_channel(enum mcp3426_channel channel)
{
    assert(!msg_in_progress);
    assert(mode == MCP3426_MODE_CONTINOUS || mode == MCP3426_MODE_ONE_SHOT);
    assert(channel < MCP3426_CHANNEL_COUNT);

    set_channel_req.tx_buffer = &set_channel_cmds[channel];
    set_channel_req.bytes_to_send = 1;
    async_i2c_enqueue(&set_channel_req, &msg_in_progress);
}

static void mcp3426_on_setup_complete(const struct async_i2c_request *req)
{
    mode = (int) req->user_data;
    mcp3426_set_channel(0);
}

static void mcp3426_on_read(const struct async_i2c_request *req)
{
    (void) req;
    uint16_t data = rx_buffer[0];
    data = ((data << 8) & 0xFF00) | rx_buffer[1];
    uint8_t config = rx_buffer[2];
    uint8_t channel = (config >> 5) & 0x03;
    bool ready = !(config & 0x80);

    if(ready) {
        float voltage = (float) data;
        voltage *= 2.048 * 2.0;
        voltage /= 65536.0;

        adc_values[channel] = voltage;

        if(channel == MCP3426_CHANNEL_COUNT - 1)  {
            mcp3426_set_channel(0);
        } else {
            mcp3426_set_channel(channel + 1);
        }
    } else {
        add_alarm_in_ms(100, &mcp3426_enqueue_read, NULL, true);
    }
}

/**
 * Initializes the driver and resets the MCP3426
*/
void mcp3426_init(int adc_mode, enum mcp3426_sample_rate rate, enum mcp3426_gain gain)
{
    assert(mode == MCP3426_MODE_UNINITIALIZED);
    assert(!msg_in_progress);

    for (int i = 0; i < MCP3426_CHANNEL_COUNT; i++)
    {
        set_channel_cmds[i] = mcp3426_config_reg(MCP3426_START_READ, i, adc_mode, rate, gain);
    }

    static_assert(sizeof(mode) <= sizeof(general_call_reset_req.user_data));
    general_call_reset_req.completed_callback = &mcp3426_on_setup_complete;
    general_call_reset_req.user_data = (void*) mode;

    mode = MCP3426_MODE_INITIALIZING;
    async_i2c_enqueue(&general_call_reset_req, &msg_in_progress);
}

/**
 * Reads the most recent voltage returned by the ADC
 *
 * If not value has been read yet, zero is returned.
*/
float mcp3426_read_voltage(enum mcp3426_channel channel)
{
    assert(channel < MCP3426_CHANNEL_COUNT);

    return adc_values[channel];
}