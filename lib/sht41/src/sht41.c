#include "async_i2c.h"

#define SHT41_I2C_ADDR 0x44
#define SHT41_WRITE_CMD 0xFD
#define SHT41_READ_LENGTH 6

#define SHT41_POLL_TIME_MS 10

static void on_write(const struct async_i2c_request *req);

static void on_read(const struct async_i2c_request *req);

static void on_error();

static const uint8_t read_cmd_tx[] = {SHT14_WRITE_CMD};
static const uint8_t read_cmd_rx[SHT41_READ_LENGTH] = {0};

static uint rx_buf[SHT14_READ_LENGTH];
static struct async_i2c_request read_req = {
    .i2c_num = 0,
    .address = SHT14_I2C_ADDR,
    .nostop = false,
    .tx_buffer = read_cmd_tx,
    .rx_buffer = read_cmd_rx,
    .bytes_to_send = 1,
    .bytes_to_receive = SHT14_READ_LENGTH,
    .completed_callback = on_read,
    .failed_callback = on_error,
    .next_req_on_success = NULL
}; 

void 