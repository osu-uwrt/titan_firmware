#include "sht41.h"
#include "async_i2c.h"

#define SHT14_I2C_ADDR 0x44
#define SHT14_WRITE_CMD 0xFD
#define SHT14_READ_LENGTH 6

static void on_write(const struct async_i2c_request *req);

static void on_read(const struct async_i2c_request *req);

static void on_error();

static const uint8_t write_cmd[] = {SHT14_WRITE_CMD};
static struct async_i2c_request write_req = {
    .i2c_num = 0,
    .address = SHT14_I2C_ADDR,
    .nostop = false,
    .tx_buffer = &write_cmd,
    .rx_buffer = NULL,
    .bytes_to_send = sizeof(write_cmd),
    .bytes_to_receive = 0,
    .completed_callback = on_write,
    .failed_callback = on_error,
    .next_req_on_success = NULL
};

static uint rx_buf[SHT14_READ_LENGTH];
static struct async_i2c_request read_req = {
    .i2c_num = 0,
    .address = SHT14_I2C_ADDR,
    .nostop = false,
    .tx_buffer = NULL,
    .rx_buffer = NULL,
    .bytes_to_send = 0,
    .bytes_to_receive = SHT14_READ_LENGTH,
    .completed_callback = on_read,
    .failed_callback = on_error,
    .next_req_on_success = NULL
}; 

static bool msg_in_progress = false;

// TODO: Add error string

static void start_read_callback(__unused alarm_id_t id, __unused void *user_data) {
    async_i2c_enqueue(&write_req, &msg_in_progress);
 }

static void on_write(const struct async_i2c_request *req) { 
    add_alarm_in_ms(10, &start_read_callback, NULL, true);
}


static void on_read(const struct async_i2c_request *req) { 
    
}

bool sht41_init() { 
    return false;
}

bool sht41_read_async(struct sht41_read_req* req) { 
    write_req.user_data = req;
    return async_i2c_enqueue(&write_req, &msg_in_progress);
}