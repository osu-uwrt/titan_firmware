#include "driver/sht41.h"

#include "driver/async_i2c.h"

#define SHT41_I2C_ADDR 0x44
#define SHT41_WRITE_CMD 0xFD
#define SHT41_READ_LENGTH 6
#define TIME_MEASURE 10

#define SHT41_POLL_TIME_MS 10

static void on_write(const struct async_i2c_request *req);

static void on_read(const struct async_i2c_request *req);

static void on_error();

static const uint8_t write_cmd[] = { SHT41_WRITE_CMD };
static struct async_i2c_request write_req = { .i2c_num = 0,
                                              .address = SHT41_I2C_ADDR,
                                              .nostop = false,
                                              .tx_buffer = write_cmd,
                                              .rx_buffer = NULL,
                                              .bytes_to_send = sizeof(write_cmd),
                                              .bytes_to_receive = 0,
                                              .completed_callback = on_write,
                                              .failed_callback = on_error,
                                              .next_req_on_success = NULL };

static uint8_t rx_buf[SHT41_READ_LENGTH];
static struct async_i2c_request read_req = { .i2c_num = 0,
                                             .address = SHT41_I2C_ADDR,
                                             .nostop = false,
                                             .tx_buffer = NULL,
                                             .rx_buffer = rx_buf,
                                             .bytes_to_send = 0,
                                             .bytes_to_receive = SHT41_READ_LENGTH,
                                             .completed_callback = on_read,
                                             .failed_callback = on_error,
                                             .next_req_on_success = NULL };

static bool msg_in_progress = false;

// TODO: Add error string

static void start_read_callback(__unused alarm_id_t id, void *user_data) {
    // async_i2c_enqueue(&write_req, &msg_in_progress);

    read_req.user_data = user_data;                  // kind of unnecessary when global
                                                     // variabl write_req already hold user_data
    async_i2c_enqueue(&read_req, &msg_in_progress);  // when i2c done read, it calls on_read()
}

static void on_write(const struct async_i2c_request *req) {
    add_alarm_in_ms(TIME_MEASURE, &start_read_callback, req->user_data, true);
}

static void on_read(const struct async_i2c_request *req) {
    // the user_data of type sht41_read_req is started from sht41_read_async
    struct sht41_read_req *on_read_read_req = (struct sht41_read_req *) req->user_data;
    on_read_read_req->data->temp = req->rx_buffer[0] << 8 | req->rx_buffer[1];
    // CRC check for temp data
    uint8_t temp_CRC = req->rx_buffer[0];

    on_read_read_req->data->rh = req->rx_buffer[3] << 8 | req->rx_buffer[4];
    // CRC check for rh data
    uint8_t rh_CRC = req->rx_buffer[5];

    // need timestamp until invalid

    // after processing data, callback to wherever asked for sht41 data, Correct?
    on_read_read_req->callback();
}

// called by the i2c
static void on_error(const struct async_i2c_request *req) {
    // process error
    // decode abort_data
    // error_callback
}

// bool sht41_init() {
//     return false;
// }

// alarm_sampling_timer has to call this shtt41_read_async
// and alarm_sampling_timer is created from driver_sht41_init
// in the init() create the global variable sht41_data
bool sht41_read_async(struct sht41_read_req *req) {
    write_req.user_data = req;

    return async_i2c_enqueue(&write_req, &msg_in_progress);
}
