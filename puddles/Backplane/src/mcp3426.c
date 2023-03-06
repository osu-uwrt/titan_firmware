#include <mcp3426.h>
#include <async_i2c.h>
#include "pico/stdlib.h"

uint16_t mcp3426_value_array[CHANNEL_NUM] = {0, 0, 0, 0};
uint8_t mcp3426_valid_array[CHANNEL_NUM] = {0, 0, 0, 0};

void mcp3426_init() {
    uint8_t data[1];
    for(uint8_t chan = 0; chan < CHANNEL_NUM; chan++) {
        data[0] = 0xFF & mcp3426_config_reg(chan, 0, 0);
        async_i2c_write_blocking_until(i2c1, MCP3426_ADDR, data, 1, 0, make_timeout_time_ms(5));
    }
}

void mcp3426_read() {
    //Cycle through the channels and get their status
    uint8_t data[3];
    for(uint8_t chan = 0; chan < CHANNEL_NUM; chan++) {
        if(!mcp3426_valid_array[chan]) {
            data[0] = mcp3426_config_reg(chan, 0, 0);
            async_i2c_write_blocking_until(i2c1, MCP3426_ADDR, data, 1, 0, make_timeout_time_ms(5));
            busy_wait_us(7500);
            async_i2c_read_blocking_until(i2c1, MCP3426_ADDR, data, 3, false, make_timeout_time_ms(5));
            if(!(data[2] & 0x10000000)) {
                mcp3426_value_array[chan] = ((*data << 8) & 0xff00 | (*(data + 1) & 0x00ff));
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