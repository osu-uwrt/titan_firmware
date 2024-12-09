#ifndef DRIVER__MCP3426_H_
#define DRIVER__MCP3426_H_

#include "driver/async_i2c.h"

#include <stdbool.h>
#include <stdint.h>

// PICO_CONFIG: PARAM_ASSERTIONS_ENABLED_DRIVER_MCP3426, Enable/disable assertions in the MCP3426 Driver, type=bool, default=0, group=driver_mcp3426
#ifndef PARAM_ASSERTIONS_ENABLED_DRIVER_MCP3426
#define PARAM_ASSERTIONS_ENABLED_DRIVER_MCP3426 0
#endif

enum mcp3426_channel {
    MCP3426_CHANNEL_1 = 0,
    MCP3426_CHANNEL_2 = 1,
    MCP3426_CHANNEL_3 = 2,
    MCP3426_CHANNEL_4 = 3,
    MCP3426_CHANNEL_COUNT,
};

typedef void (*mcp3426_error_cb)(const struct async_i2c_request *req, uint32_t error_code);

/**
 * @brief Initializes the mcp3426 driver
 *
 * @param i2c_num The i2c bus number for the mcp3426
 * @param addr The i2c address for the mcp3426
 * @param mcp3426_error_cb Callback on error (can be NULL)
 */
void mcp3426_init(uint8_t i2c_num, uint8_t addr, mcp3426_error_cb error_cb);

/**
 * @brief Returns the raw voltage read on the ADC pin, or NaN if no reading available
 *
 * @param channel ADC channel to report
 * @return float The voltage on the pin, or NaN if no reading available
 */
float mcp3426_read_voltage(enum mcp3426_channel channel);

#endif
