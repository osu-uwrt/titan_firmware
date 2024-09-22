/**
 * @file bq25730.c
 * @brief Driver for the BQ25730 story.
 */

#include "bq25730.h"

#include "driver/async_i2c.h"
#include <stdio.h>

#define BQ25730_I2C_TOTAL_REGISTERS_SIZE 64

static struct async_i2c_request i2c_req;

static uint8_t i2c_rx_buffer[BQ25730_I2C_TOTAL_REGISTERS_SIZE];
static uint8_t i2c_tx_buffer[BQ25730_I2C_TOTAL_REGISTERS_SIZE];

static bool i2c_request_in_progress = false;

typedef enum bq25730_register {
    BQ25730_REG_CHARGER_STATUS = 0x20,
    BQ25730_REG_MANUFACTURER_ID = 0x2E,
    // .. //
} bq25730_register_t;

static void bq25730_on_register_read(bq25730_register_t reg, uint8_t *data, size_t data_len);

static void bq25730_read_data(bq25730_register_t reg) {
    i2c_req.bytes_to_send = 1;
    i2c_tx_buffer[0] = (uint8_t) reg;
    i2c_req.bytes_to_receive = 1;
    i2c_req.timeout = make_timeout_time_ms(1000);
    async_i2c_enqueue(&i2c_req, &i2c_request_in_progress);
}

static void bq25730_write_data(bq25730_register_t reg_start, uint8_t *data, size_t data_len) {
    (void) reg_start;
    (void) data;
    (void) data_len;
}

static void bq25730_on_i2c_failure(const struct async_i2c_request *req, uint32_t error) {
    printf("Error: %lu\n", error);
}

static void bq25730_on_i2c_req_complete(const struct async_i2c_request *req) {
    printf("Manufacturer ID: %x\n", i2c_rx_buffer[0]);
}

void bq25730_init(unsigned int busNum) {
    i2c_req.i2c_num = busNum;
    i2c_req.address = 0x6b;
    i2c_req.tx_buffer = i2c_tx_buffer;
    i2c_req.rx_buffer = i2c_rx_buffer;
    i2c_req.bytes_to_receive = 0;
    i2c_req.bytes_to_send = 0;
    i2c_req.completed_callback = bq25730_on_i2c_req_complete;
    i2c_req.failed_callback = bq25730_on_i2c_failure;
    i2c_req.next_req_on_success = NULL;
}

void bq25730_start_read_manufacturer_id() {
    bq25730_read_data(BQ25730_REG_MANUFACTURER_ID);
}
