#ifndef MCP3426
#define MCP3426

enum mcp3426_gain {
    MCP3426_GAIN_X1 = 0,
    MCP3426_GAIN_X2 = 1,
    MCP3426_GAIN_X4 = 2,
    MCP3426_GAIN_X8 = 3,
};

enum mcp3426_channel {
    MCP3426_CHANNEL_1 = 0,
    MCP3426_CHANNEL_2 = 1,
    MCP3426_CHANNEL_3 = 2,
    MCP3426_CHANNEL_4 = 3,
    MCP3426_CHANNEL_COUNT,
};

enum mcp3426_sample_rate {
    MCP3426_SAMPLE_RATE_12_BIT = 0,
    MCP3426_SAMPLE_RATE_14_BIT = 1,
    MCP3426_SAMPLE_RATE_16_BIT = 2,
};

void mcp3426_init(void);

int mcp3426_read(enum mcp3426_channel channel);

#endif