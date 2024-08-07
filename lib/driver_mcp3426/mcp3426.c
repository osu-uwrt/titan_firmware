#include "driver/mcp3426.h"

#include "driver/async_i2c.h"

#include <math.h>

// Modes used internally by the driver
#define MCP3426_MODE_UNINITIALIZED 2
#define MCP3426_MODE_INITIALIZING 3

#define MCP3426_START_READ 1
#define MCP3426_WAIT_TO_READ 0

enum mcp3426_gain {
    MCP3426_GAIN_X1 = 0,
    MCP3426_GAIN_X2 = 1,
    MCP3426_GAIN_X4 = 2,
    MCP3426_GAIN_X8 = 3,
};

enum mcp3426_sample_rate {
    MCP3426_SAMPLE_RATE_12_BIT = 0,
    MCP3426_SAMPLE_RATE_14_BIT = 1,
    MCP3426_SAMPLE_RATE_16_BIT = 2,
};

enum mcp3426_mode {
    MCP3426_MODE_ONE_SHOT = 0,
    MCP3426_MODE_CONTINOUS = 1,
};

/// Creates a MCP3426 configuration byte.
#define mcp3426_config_reg(read, chan, mode, rate, gain) (read << 7) | (chan << 5) | (mode << 4) | (rate << 2) | (gain)

static void mcp3426_on_read(const struct async_i2c_request *req);
static void mcp3426_on_write(const struct async_i2c_request *req);
static void mcp3426_on_setup_complete(const struct async_i2c_request *req);

static mcp3426_error_cb error_callback;
static float adc_values[MCP3426_CHANNEL_COUNT] = { [0 ... MCP3426_CHANNEL_COUNT - 1] = NAN };
static int mode = MCP3426_MODE_UNINITIALIZED;
static bool msg_in_progress = false;
static int err_count = 2;  // Start where 1 error will cause fault, cleared to 0 on first successful read

static void mcp3426_on_error(const struct async_i2c_request *req, uint32_t error_code) {
    err_count++;
    if (err_count >= 3) {
        if (error_callback) {
            error_callback(req, error_code);
        }

        for (size_t i = 0; i < MCP3426_CHANNEL_COUNT; i++) {
            adc_values[i] = NAN;
        }
    }
}

static uint8_t set_channel_cmds[MCP3426_CHANNEL_COUNT] = { 0 };
static struct async_i2c_request set_channel_req = {
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
    .nostop = false,
    .tx_buffer = &general_call_reset_cmd,
    .rx_buffer = NULL,
    .bytes_to_send = 1,
    .bytes_to_receive = 0,
    .completed_callback = &mcp3426_on_setup_complete,
    .failed_callback = &mcp3426_on_error,
    .next_req_on_success = NULL,
};

static uint8_t rx_buffer[3];
static struct async_i2c_request read_req = {
    .nostop = false,
    .tx_buffer = NULL,
    .rx_buffer = rx_buffer,
    .bytes_to_send = 0,
    .bytes_to_receive = 3,
    .completed_callback = mcp3426_on_read,
    .failed_callback = &mcp3426_on_error,
    .next_req_on_success = NULL,
};

static int64_t mcp3426_enqueue_read(__unused alarm_id_t id, __unused void *user_data) {
    async_i2c_enqueue(&read_req, &msg_in_progress);

    return 0;
}

static void mcp3426_on_write(const struct async_i2c_request *req) {
    (void) req;
    add_alarm_in_ms(100, &mcp3426_enqueue_read, NULL, true);
}

static void mcp3426_set_channel(enum mcp3426_channel channel) {
    assert(!msg_in_progress);
    assert(mode == MCP3426_MODE_CONTINOUS || mode == MCP3426_MODE_ONE_SHOT);
    assert(channel < MCP3426_CHANNEL_COUNT);

    set_channel_req.tx_buffer = &set_channel_cmds[channel];
    set_channel_req.bytes_to_send = 1;
    async_i2c_enqueue(&set_channel_req, &msg_in_progress);
}

static void mcp3426_on_setup_complete(const struct async_i2c_request *req) {
    mode = (int) req->user_data;
    mcp3426_set_channel(0);
}

static void mcp3426_on_read(const struct async_i2c_request *req) {
    (void) req;
    uint16_t data = rx_buffer[0];
    data = ((data << 8) & 0xFF00) | rx_buffer[1];
    uint8_t config = rx_buffer[2];
    uint8_t channel = (config >> 5) & 0x03;
    bool ready = !(config & 0x80);

    err_count = 0;

    if (ready) {
        float voltage = (float) data;
        voltage *= 2.048 * 2.0;
        voltage /= 65536.0;

        adc_values[channel] = voltage;

        if (channel == MCP3426_CHANNEL_COUNT - 1) {
            mcp3426_set_channel(0);
        }
        else {
            mcp3426_set_channel(channel + 1);
        }
    }
    else {
        add_alarm_in_ms(100, &mcp3426_enqueue_read, NULL, true);
    }
}

/**
 * Initializes the driver and resets the MCP3426
 */
void mcp3426_init(uint8_t i2c_num, uint8_t addr, mcp3426_error_cb error_cb) {
    // Load requests with i2c number and address
    set_channel_req.i2c_num = i2c_num;
    set_channel_req.address = addr;
    general_call_reset_req.i2c_num = i2c_num;
    general_call_reset_req.address = addr;
    read_req.i2c_num = i2c_num;
    read_req.address = addr;

    // Configuration
    error_callback = error_cb;
    int adc_mode = MCP3426_MODE_CONTINOUS;
    enum mcp3426_sample_rate rate = MCP3426_SAMPLE_RATE_16_BIT;
    enum mcp3426_gain gain = MCP3426_GAIN_X1;

    hard_assert_if(DRIVER_MCP3426, mode != MCP3426_MODE_UNINITIALIZED);
    hard_assert_if(DRIVER_MCP3426, msg_in_progress);

    for (int i = 0; i < MCP3426_CHANNEL_COUNT; i++) {
        set_channel_cmds[i] = mcp3426_config_reg(MCP3426_START_READ, i, adc_mode, rate, gain);
    }

    static_assert(sizeof(adc_mode) <= sizeof(general_call_reset_req.user_data));
    general_call_reset_req.completed_callback = &mcp3426_on_setup_complete;
    general_call_reset_req.user_data = (void *) adc_mode;

    mode = MCP3426_MODE_INITIALIZING;
    async_i2c_enqueue(&general_call_reset_req, &msg_in_progress);
}

/**
 * Reads the most recent voltage returned by the ADC
 *
 * If not value has been read yet, zero is returned.
 */
float mcp3426_read_voltage(enum mcp3426_channel channel) {
    valid_params_if(DRIVER_MCP3426, channel < MCP3426_CHANNEL_COUNT);

    return adc_values[channel];
}
