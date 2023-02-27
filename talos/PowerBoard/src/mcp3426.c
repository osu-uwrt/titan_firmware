#include "mcp3426.h"
#include "pico/stdlib.h"
#include "stdint.h"
#include "async_i2c.h"

#define MCP3426_WRITE_ADDR 0x68 // 1101000
#define MCP3426_READ_ADDR 0x69  // 1101001 
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

static uint8_t set_channel_cmds[MCP3426_CHANNEL_COUNT] = {0};
static struct async_i2c_request set_channel_req = {
    .i2c_num = I2C_NUM,
    .address = MCP3426_WRITE_ADDR,
    .nostop = false,
    .tx_buffer = NULL, 
    .rx_buffer = NULL,
    .bytes_to_send = 0,
    .bytes_to_receive = 0,
    .completed_callback = NULL,
    .failed_callback = NULL,
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
    .completed_callback = NULL,
    .failed_callback = NULL, // TODO
    .next_req_on_success = NULL,
};

static uint8_t rx_buffer[3];
static struct async_i2c_request read_req = {
    .i2c_num = I2C_NUM,
    .address = MCP3426_READ_ADDR,
    .nostop = false,
    .tx_buffer = NULL, 
    .rx_buffer = rx_buffer,
    .bytes_to_send = 0,
    .bytes_to_receive = 3,
    .completed_callback = mcp3426_on_read,
    .failed_callback = NULL,
    .next_req_on_success = NULL,
};
static int mode = MCP3426_MODE_UNINITIALIZED;

static bool read_in_progress = false;
static bool write_in_progress = false;

static uint16_t adc_values[MCP3426_CHANNEL_COUNT] = {0};

void mcp3426_set_channel(enum mcp3426_channel channel)
{
    assert(!write_in_progress);
    assert(mode == MCP3426_MODE_CONTINOUS || mode == MCP3426_MODE_ONE_SHOT);
    assert(channel >= MCP3426_CHANNEL_1 && channel < MCP3426_CHANNEL_COUNT);

    // TODO: Support setting channel but not starting a read. 
    set_channel_req.tx_buffer = &set_channel_cmds[channel];
    set_channel_req.bytes_to_send = 1;
    async_i2c_enqueue(&set_channel_req, &write_in_progress);
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
        adc_values[channel] = data;
        printf("[ADC %d]: %hd\n", channel, data);

        if(mode == MCP3426_MODE_CONTINOUS) { 
            if(channel == MCP3426_CHANNEL_COUNT - 1)  {
                mcp3426_set_channel(0);
            } else {
                mcp3426_set_channel(channel + 1);
            }
        }
    }
}

/**
 * Initializes the driver and resets the MCP3426
*/
void mcp3426_init(int adc_mode, enum mcp3426_sample_rate rate, enum mcp3426_gain gain)
{
    assert(mode == MCP3426_MODE_UNINITIALIZED);
    assert(!read_in_progress);
    assert(!write_in_progress);

    for (int i = 0; i < MCP3426_CHANNEL_COUNT; i++)
    {
        set_channel_cmds[i] = mcp3426_config_reg(MCP3426_START_READ, i, adc_mode, rate, gain);
    }

    static_assert(sizeof(mode) <= sizeof(general_call_reset_req.user_data));
    general_call_reset_req.completed_callback = &mcp3426_on_setup_complete;
    general_call_reset_req.user_data = (void*) mode;

    mode = MCP3426_MODE_INITIALIZING;
    async_i2c_enqueue(&general_call_reset_req, &read_in_progress);
}

/**
 * This function will start a new read request if one is not in progress. 
 * 
 * Note: This function must be called after mcp_3426_init()
*/
void mcp3426_update()
{
    assert(mode == MCP3426_MODE_CONTINOUS || mode == MCP3426_MODE_ONE_SHOT);

    if(!read_in_progress) { 
        async_i2c_enqueue(&read_req, &read_in_progress);
    } 
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
