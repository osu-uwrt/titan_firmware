#include "driver/sht41.h"

#include "driver/async_i2c.h"

#define SHT41_I2C_ADDR 0x44
#define SHT41_WRITE_CMD 0xFD
#define SHT41_READ_LENGTH 6

#define SHT41_SAMPLING_TIME_MS 10
#define SHT41_POLL_TIME_MS 500
#define SHT41_INVALID_TIMEOUT_MS 600

static void on_write(const struct async_i2c_request *req);

static void on_read(const struct async_i2c_request *req);

static void on_error();

static const uint8_t write_cmd[] = { SHT41_WRITE_CMD };
static uint8_t rx_buf[SHT41_READ_LENGTH];
// static struct async_i2c_request read_req = { .i2c_num = 0,  // FIXME put in i2c_num
//                                              .address = SHT41_I2C_ADDR,
//                                              .nostop = false,
//                                              .tx_buffer = NULL,
//                                              .rx_buffer = rx_buf,
//                                              .bytes_to_send = 0,
//                                              .bytes_to_receive = SHT41_READ_LENGTH,
//                                              .completed_callback = on_read,
//                                              .failed_callback = on_error,
//                                              .next_req_on_success = NULL };

static struct sht41_data {
    int16_t temp;
    int16_t rh;
    absolute_time_t reading_expiration;
    struct async_i2c_request write_read_req;  // TODO nned to init in the init()
} last_reading;

static bool msg_in_progress = false;

static sht41_error_cb error_cb;

// TODO: Add error string

static int16_t start_read_callback(__unused alarm_id_t id, void *user_data) {
    // async_i2c_enqueue(&write_req, &msg_in_progress);
    // FIXME make sure code is maintenacable
    last_reading.write_read_req.user_data = user_data;
    last_reading.write_read_req.completed_callback = on_read;
    async_i2c_enqueue(&last_reading.write_read_req, &msg_in_progress);  // when i2c done read, it calls on_read()

    return 0;
}

static void on_write(const struct async_i2c_request *req) {
    if (add_alarm_in_ms(SHT41_SAMPLING_TIME_MS, &start_read_callback, req->user_data, true) < 0) {
        // TODO: Check that alarm scheduled if not success, err callback
    }
}

static void on_read(const struct async_i2c_request *req) {
    // the user_data of type sht41_read_req is started from sht41_read_async
    struct sht41_data *sampled_data = (struct sht41_data *) req->user_data;

    // CRC check for temp data
    uint8_t temp_CRC = req->rx_buffer[0];

    sampled_data->temp = req->rx_buffer[0] << 8 | req->rx_buffer[1];

    // CRC check for rh data
    uint8_t rh_CRC = req->rx_buffer[5];

    sampled_data->rh = req->rx_buffer[3] << 8 | req->rx_buffer[4];

    // need timestamp after this
    sampled_data->reading_expiration = make_timeout_time_ms(SHT41_INVALID_TIMEOUT_MS);
}

// called by the i2c
static void on_error(const struct async_i2c_request *req) {
    // process error
    // decode abort_data from i2c to get error_data
    // error callback to board: error_cb(error_data)
}

void sht41_init(sht41_error_cb board_error_cb) {
    error_cb = board_error_cb;
    last_reading.write_read_req.i2c_num = 0;  // TODO get i2c_num
    last_reading.write_read_req.address = SHT41_I2C_ADDR;
    last_reading.write_read_req.nostop = false;
    last_reading.write_read_req.tx_buffer = write_cmd;
    last_reading.write_read_req.rx_buffer = rx_buf;
    last_reading.write_read_req.bytes_to_send = sizeof(write_cmd);
    last_reading.write_read_req.bytes_to_receive = SHT41_READ_LENGTH;
    last_reading.write_read_req.failed_callback = on_error;
    last_reading.write_read_req.next_req_on_success = NULL;
    if (add_alarm_in_ms(SHT41_POLL_TIME_MS, start_sht41_read_async, &last_reading, true) < 0) {
        // TODO: Call err cb
    }
}

// alarm_sampling_timer has to call this shtt41_read_async
// and alarm_sampling_timer is created from driver_sht41_init
// in the init() create the global variable sht41_data
static int64_t start_sht41_read_async(__unused alarm_id_t id, void *user_data) {
    struct sht41_data *local = (struct sht41_data *) user_data;
    local->write_read_req.completed_callback = on_write;
    local->write_read_req.user_data = user_data;  // FIXME not sure
    async_i2c_enqueue(&(local->write_read_req), &msg_in_progress);
    return SHT41_POLL_TIME_MS * 1000;
}

// User function called by the board to get data

bool sht41_is_valid(void) {
    return time_reached(last_reading.reading_expiration);
}

int16_t sht41_read_temp(void) {
    return last_reading.temp;
}

int16_t sht41_read_rh(void) {
    return last_reading.rh;
}
