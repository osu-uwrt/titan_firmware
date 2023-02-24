#ifndef MCP3426
#define MCP3426

#include "hardware/i2c.h"
#include "pico/stdlib.h"

#define MCP3426_ADDR 0x68
#define mcp3426_config_reg(chan, rate, gain) 0x80 | (chan << 5) | (rate << 2) | (gain)

uint8_t mcp3426_gain_array[4] = {0, 0, 0, 0};

void mcp3426_init() {
    i2c_init(i2c1, 100000);
    gpio_set_function(BOARD_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(BOARD_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(BOARD_SDA_PIN);
    gpio_pull_up(BOARD_SCL_PIN);
}

void mcp3426_read() {
    //Cycle through the channels and get their status
    uint8_t data[2];
    for(uint8_t chan = 0; chan < 4; chan++) {
        data[0] = 0x0000FFFF & mcp3426_config_reg(chan, 0, mcp3426_gain_array[chan]);
        i2c_write_blocking(i2c1, MCP3426_ADDR, data, 1, 0);
        sleep_us(5000);
        i2c_read_blocking(i2c1, MCP3426_ADDR, data, 4, false);
        sleep_us(500);
    }
}

#endif