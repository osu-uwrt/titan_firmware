#ifndef MCP3426_H
#define MCP3426_H

#include "hardware/i2c.h"
#include "pico/stdlib.h"

#define CHANNEL_NUM 4

#define MCP3426_ADDR 0x68
#define mcp3426_config_reg(chan, rate, gain) 0x90 | (chan << 5) | (rate << 2) | (gain)

void mcp3426_init();
void mcp3426_read();
uint16_t mcp3426_query(uint8_t chan);

#endif