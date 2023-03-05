#include "mcp3426.h"
#include "pico/stdlib.h"
#include "stdint.h"
#include "async_i2c.h"

// Configuration for the driver
#define MCP3426_POLL_RATE_MS 100  // Based on the sample rate
#define MCP3426_CONFIGURED_SAMPLE_RATE MCP3426_SAMPLE_RATE_16_BIT
#define MCP3426_CONFIGURED_GAIN MCP3426_GAIN_X1
#define MCP3426_CONFIGURED_MODE MCP3426_MODE_CONTINOUS

#define I2C_BUS_NUM BOARD_I2C
#define MCP3426_ADDR 0x34 // 110100
#define MCP3426_GENERAL_CALL_RESET 0x06

// Modes used internally by the driver
enum mcp3426_mode {
    MCP3426_MODE_ONE_SHOT = 0,
    MCP3426_MODE_CONTINOUS = 1,
    MCP3426_MODE_UNINITIALIZED = 2,
    MCP3426_MODE_INITIALIZING = 3
};

#define MCP3426_START_READ 1
#define MCP3426_WAIT_TO_READ 0


/// Creates a MCP3426 configuration byte.
#define mcp3426_config_reg(read, chan, mode, rate, gain) (read << 7) | (chan << 5) | (mode << 4) | (rate << 2) | (gain)

static void mcp3426_on_read(const struct async_i2c_request *req);
static void mcp3426_on_write(const struct async_i2c_request *req);
static void mcp3426_on_setup_complete(const struct async_i2c_request *req);

static void mcp3426_failed_cb(const struct async_i2c_request *req, uint32_t data) {
    printf("Failed on request %p: 0x%08x\n", req, data);
}

static uint8_t set_channel_cmds[MCP3426_CHANNEL_COUNT] = {0};
static struct async_i2c_request set_channel_req = {
    .i2c_num = I2C_BUS_NUM,
    .address = MCP3426_ADDR,
    .nostop = false,
    .tx_buffer = NULL,
    .rx_buffer = NULL,
    .bytes_to_send = 0,
    .bytes_to_receive = 0,
    .completed_callback = &mcp3426_on_write,
    .failed_callback = &mcp3426_failed_cb,
    .next_req_on_success = NULL,
};

static uint8_t general_call_reset_cmd = 0x06;
static struct async_i2c_request general_call_reset_req = {
    .i2c_num = I2C_BUS_NUM,
    .address = 0x00,  // Note: If this causes errors, switch to a general call
    .nostop = false,
    .tx_buffer = &general_call_reset_cmd,
    .rx_buffer = NULL,
    .bytes_to_send = 1,
    .bytes_to_receive = 0,
    .completed_callback = &mcp3426_on_setup_complete,
    .failed_callback = &mcp3426_failed_cb, // TODO
    .next_req_on_success = NULL,
};

static uint8_t rx_buffer[3];
static struct async_i2c_request read_req = {
    .i2c_num = I2C_BUS_NUM,
    .address = MCP3426_ADDR,
    .nostop = false,
    .tx_buffer = NULL,
    .rx_buffer = rx_buffer,
    .bytes_to_send = 0,
    .bytes_to_receive = 3,
    .completed_callback = mcp3426_on_read,
    .failed_callback = &mcp3426_failed_cb,
    .next_req_on_success = NULL,
};
static int mode = MCP3426_MODE_UNINITIALIZED;

static bool msg_in_progress = false;

static uint16_t adc_values[MCP3426_CHANNEL_COUNT] = {0};

static int64_t mcp3426_enqueue_read(__unused alarm_id_t id, __unused void *user_data) {
    async_i2c_enqueue(&read_req, &msg_in_progress);

    return 0;
}

static void mcp3426_on_write(const struct async_i2c_request *req) {
    (void) req;
    hard_assert(add_alarm_in_ms(100, &mcp3426_enqueue_read, NULL, true) > 0);
}

static void mcp3426_set_channel(enum mcp3426_channel channel)
{
    assert(!msg_in_progress);
    assert(mode == MCP3426_MODE_CONTINOUS || mode == MCP3426_MODE_ONE_SHOT);
    assert(channel >= MCP3426_CHANNEL_1 && channel < MCP3426_CHANNEL_COUNT);

    set_channel_req.tx_buffer = &set_channel_cmds[channel];
    set_channel_req.bytes_to_send = 1;
    async_i2c_enqueue(&set_channel_req, &msg_in_progress);
}

static void mcp3426_on_setup_complete(const struct async_i2c_request *req)
{
    mode = MCP3426_CONFIGURED_MODE;
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
        adc_values[channel] = data;
        printf("[ADC %d]: %hd\n", channel, data);

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
void mcp3426_init(void)
{
    assert(mode == MCP3426_MODE_UNINITIALIZED);
    assert(!msg_in_progress);

    for (int i = 0; i < MCP3426_CHANNEL_COUNT; i++)
    {
        set_channel_cmds[i] = mcp3426_config_reg(MCP3426_START_READ, i, MCP3426_CONFIGURED_MODE, MCP3426_CONFIGURED_SAMPLE_RATE, MCP3426_CONFIGURED_GAIN);
    }

    mode = MCP3426_MODE_INITIALIZING;
    async_i2c_enqueue(&general_call_reset_req, &msg_in_progress);
}

/**
 * Reads the most recent value returned by the ADC.
 *
 * If not value has been read yet, zero is returned.
*/
int mcp3426_read(enum mcp3426_channel channel)
{
    assert(channel >= MCP3426_CHANNEL_1 && channel < MCP3426_CHANNEL_COUNT);

    return adc_values[channel];
}