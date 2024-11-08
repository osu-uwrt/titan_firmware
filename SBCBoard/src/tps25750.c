/**
 * @file tps25750.c
 * @brief Driver for the TPS25750 story.
 */

#include "tps25750.h"

#include "driver/async_i2c.h"
#include <stdio.h>
#include <string.h>

#define TPS25750_I2C_TOTAL_REGISTERS_SIZE 64

static struct async_i2c_request i2c_req;

static uint8_t i2c_rx_buffer[TPS25750_I2C_TOTAL_REGISTERS_SIZE];
static uint8_t i2c_tx_buffer[TPS25750_I2C_TOTAL_REGISTERS_SIZE];

static bool i2c_request_in_progress = false;

typedef enum tps25750_register {
    TPS25750_REG_MODE = 0x03,
    TPS25750_REG_PORT_CONTROL = 0x29,
    TPS25750_REG_TX_SOURCE_CAPS = 0x32

    // .. //
} tps25750_register_t;

static void tps25750_read_data(tps25750_register_t reg) {
    i2c_req.bytes_to_send = 1;
    i2c_tx_buffer[0] = (uint8_t) reg;
    i2c_req.bytes_to_receive = 1;
    i2c_req.timeout = make_timeout_time_ms(1000);
    async_i2c_enqueue(&i2c_req, &i2c_request_in_progress);
}

static void tps25750_write_data(tps25750_register_t reg_start, uint8_t *data, size_t data_len) {
    i2c_req.bytes_to_receive = 0;
    i2c_req.bytes_to_send = data_len + 1;
    i2c_tx_buffer[0] = reg_start;
    memcpy(i2c_tx_buffer + 1, data, data_len);
    i2c_req.timeout = make_timeout_time_ms(1000);
    async_i2c_enqueue(&i2c_req, &i2c_request_in_progress);
}

static void tps25750_on_i2c_failure(const struct async_i2c_request *req, uint32_t error) {
    printf("Error: %lu\n", error);
}

static void tps25750_on_i2c_req_complete(const struct async_i2c_request *req) {
    printf("Result: %i\n", i2c_rx_buffer[0]);
}

void tps25750_init(unsigned int busNum) {
    i2c_req.i2c_num = busNum;
    i2c_req.address = 0x50;
    i2c_req.nostop = false;
    i2c_req.tx_buffer = i2c_tx_buffer;
    i2c_req.rx_buffer = i2c_rx_buffer;
    i2c_req.bytes_to_receive = 0;
    i2c_req.bytes_to_send = 0;
    i2c_req.completed_callback = tps25750_on_i2c_req_complete;
    i2c_req.failed_callback = tps25750_on_i2c_failure;
    i2c_req.next_req_on_success = NULL;
}

void tps25750_start_read_mode() {
    tps25750_read_data(TPS25750_REG_MODE);
}

void tps25750_start_write_port_control() {
    uint8_t data[] = {0x02, 0x00, 0x00, 0xE4};
    tps25750_write_data(TPS25750_REG_PORT_CONTROL, data, 4);
}

void tps25750_start_write_tx_source_caps() {
    uint8_t data[] = {0x02, 0x00, 0x00, 0xE4};
    tps25750_write_data(TPS25750_REG_PORT_CONTROL, data, 31);
}
