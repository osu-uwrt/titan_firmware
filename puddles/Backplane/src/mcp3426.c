#include <mcp3426.h>

uint8_t mcp3426_gain_array[CHANNEL_NUM] = {0, 0, 0, 0};
uint16_t mcp3426_value_array[CHANNEL_NUM] = {0, 0, 0, 0};
uint8_t mcp3426_valid_array[CHANNEL_NUM] = {0, 0, 0, 0};

void mcp3426_init() {
    uint8_t data[1];
    i2c_init(i2c1, 400000);
    gpio_set_function(BOARD_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(BOARD_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(BOARD_SDA_PIN);
    gpio_pull_up(BOARD_SCL_PIN);
    for(uint8_t chan = 0; chan < CHANNEL_NUM; chan++) {
        data[0] = 0xFF & mcp3426_config_reg(chan, 0, mcp3426_gain_array[chan]);
        i2c_write_blocking(i2c1, MCP3426_ADDR, data, 1, 0);
    }
}

void mcp3426_read() {
    //Cycle through the channels and get their status
    uint8_t data[3];
    for(uint8_t chan = 0; chan < CHANNEL_NUM; chan++) {
        if(!mcp3426_valid_array[chan]) {
            data[0] = mcp3426_config_reg(chan, 0, mcp3426_gain_array[chan]);
            i2c_write_blocking(i2c1, MCP3426_ADDR, data, 1, 0);
            busy_wait_us(7500);
            i2c_read_blocking(i2c1, MCP3426_ADDR, data, 3, false);
            if(!(data[2] & 0x10000000)) {
                mcp3426_value_array[chan] = *((uint16_t*)(data));
                mcp3426_valid_array[chan] = 1;
            }
        }
    }
}

uint16_t mcp3426_query(uint8_t chan) {
    if(mcp3426_valid_array[chan]) {
        mcp3426_valid_array[chan] = 0;
        return mcp3426_value_array[chan];
    }
    return 0;
}