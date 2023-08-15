#include <128D818.h>
#include <driver/async_i2c.h>
#include <math.h>

uint16_t D818_value_array[D818_CHANNEL_NUM] = { 0, 0, 0, 0 };
uint8_t D818_valid_array[D818_CHANNEL_NUM] = { 0, 0, 0, 0 };

void D818_init() {
    uint8_t init[] = { D818_REG_CONFIG_ADDR, 0x80 };  // Send the init bit
    async_i2c_write_blocking_until(i2c1, D818_ADDR, init, 2, 0, make_timeout_time_ms(5));
    sleep_ms(50);  // Required for safety

    init[0] = D818_REG_ADV_CFG_ADDR;
    init[1] = 0x40;
    async_i2c_write_blocking_until(i2c1, D818_ADDR, init, 2, 0, make_timeout_time_ms(5));

    init[0] = D818_IRQ_MASK_ADDR;
    init[1] = 0xFF;
    async_i2c_write_blocking_until(i2c1, D818_ADDR, init, 2, 0, make_timeout_time_ms(5));

    init[0] = D818_REG_CONFIG_ADDR;
    init[1] = 0x01;
    async_i2c_write_blocking_until(i2c1, D818_ADDR, init, 2, 0, make_timeout_time_ms(5));
}

void D818_read() {
    // Cycle through the channels and get their status
    uint8_t data[3];
    data[0] = D818_REG_ONESHOT_ADDR;
    data[1] = 0x01;
    async_i2c_write_blocking_until(i2c1, D818_ADDR, data, 2, 0, make_timeout_time_ms(5));
    data[0] = D818_REG_BUSY_ADDR;
    async_i2c_write_blocking_until(i2c1, D818_ADDR, data, 1, 1, make_timeout_time_ms(5));
    do {
        async_i2c_read_blocking_until(i2c1, D818_ADDR, data, 1, 0, make_timeout_time_ms(5));
    } while ((*data & 0x01) != 0);
    for (uint8_t chan = 0; chan < D818_CHANNEL_NUM; chan++) {
        if (!D818_valid_array[chan]) {
            data[0] = D818_REG_READING_ADDR | chan;
            async_i2c_write_blocking_until(i2c1, D818_ADDR, data, 1, 1, make_timeout_time_ms(5));
            async_i2c_read_blocking_until(i2c1, D818_ADDR, data, 2, 0, make_timeout_time_ms(5));
            // Do some data validation here?
            D818_value_array[chan] = *(data);
            D818_value_array[chan] = (((D818_value_array[chan] << 8)) & 0x0f00) | (((*(data + 1)) >> 4) & 0x00ff);
            D818_valid_array[chan] = 1;
        }
    }
}

float D818_query(uint8_t chan) {
    if (D818_valid_array[chan]) {
        D818_valid_array[chan] = 0;
        return D818_value_array[chan];
    }
    return NAN;
}
