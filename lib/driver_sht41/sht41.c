#include "driver/sht41.h"

#include "driver/async_i2c.h"

#include <stdio.h>

#define SHT41_WRITE_CMD 0xFD
#define SHT41_READ_LENGTH 6

#define ACCEPTABLE_ERR_COUNT 3

// Global varibles
volatile bool sht41_temp_rh_set_on_read = false;

// fnc prototypes
static int64_t sht41_start_write_req(__unused alarm_id_t id, void *user_data);

static void sht41_on_write(const struct async_i2c_request *req);

static int64_t sht41_start_read_req(__unused alarm_id_t id, void *user_data);

static void sht41_on_read(const struct async_i2c_request *req);

static void sht41_on_error(const sht41_error_code error_type);

// SHT41 data fields
static const uint8_t write_cmd[] = { SHT41_WRITE_CMD };
static uint8_t rx_buf[SHT41_READ_LENGTH];

// requests
// static struct async_i2c_request wr_req = { .i2c_num = BOARD_I2C,
//                                            .address = SHT41_I2C_ADDR,
//                                            .nostop = false,
//                                            .tx_buffer = write_cmd,
//                                            .rx_buffer = NULL,
//                                            .bytes_to_send = 1,
//                                            .bytes_to_receive = 0,
//                                            .completed_callback = &sht41_on_write,
//                                            .failed_callback = &on_async_i2c_error,
//                                            .next_req_on_success = NULL,
//                                            .user_data = NULL };

// static struct async_i2c_request rd_req = { .i2c_num = BOARD_I2C,
//                                            .address = SHT41_I2C_ADDR,
//                                            .nostop = false,
//                                            .tx_buffer = NULL,
//                                            .rx_buffer = rx_buf,
//                                            .bytes_to_send = 0,
//                                            .bytes_to_receive = 6,
//                                            .completed_callback = &sht41_on_read,
//                                            .failed_callback = &on_async_i2c_error,
//                                            .next_req_on_success = NULL,
//                                            .user_data = NULL };

static struct sht41_data {
    float temp;
    float rh;
    absolute_time_t reading_expiration;
    struct async_i2c_request read_req;
    struct async_i2c_request write_req;
    bool msg_in_progress;
} last_reading = { 0 };

static sht41_error_cb error_cb;

static uint8_t err_count = ACCEPTABLE_ERR_COUNT - 1;

static int64_t sht41_start_write_req(__unused alarm_id_t id, void *user_data) {
    // wr_req.user_data = user_data;
    // async_i2c_enqueue(&wr_req, &msg_in_progress);

    struct sht41_data *local = (struct sht41_data *) user_data;
    local->write_req.user_data = user_data;
    async_i2c_enqueue(&local->write_req, &local->msg_in_progress);

    return SHT41_POLL_TIME_MS * 1000;
}

static void sht41_on_write(const struct async_i2c_request *req) {
    if (add_alarm_in_ms(SHT41_SAMPLING_TIME_MS, &sht41_start_read_req, req->user_data, true) < 0) {
        sht41_on_error(SHT41_ERROR_TIMER_SCHEDULE_FULL);
    }
}

static uint8_t crc_check(uint8_t *data, uint8_t len) {
    uint8_t generator = 0x31;
    uint8_t crc_register = 0xff;
    for (int i = 0; i < len; i++) {
        crc_register ^= data[i];
        for (int j = 0; j < 8; j++) {
            if (crc_register & 0x80)
                crc_register = (crc_register << 1) ^ generator;
            else
                crc_register <<= 1;
        }
    }
    return crc_register;
}

static int64_t sht41_start_read_req(__unused alarm_id_t id, void *user_data) {
    // rd_req.user_data = user_data;
    // async_i2c_enqueue(&rd_req, &msg_in_progress);

    struct sht41_data *local = (struct sht41_data *) user_data;
    local->read_req.user_data = user_data;
    async_i2c_enqueue(&local->read_req, &local->msg_in_progress);
    return 0;
}

static void sht41_on_read(const struct async_i2c_request *req) {
    struct sht41_data *local = (struct sht41_data *) req->user_data;

    if (req->rx_buffer[2] != crc_check(req->rx_buffer, 2)) {
        sht41_on_error(SHT41_ERROR_CRC_INVALID);
    }
    local->temp = req->rx_buffer[0] << 8 | req->rx_buffer[1];

    if (req->rx_buffer[5] != crc_check(req->rx_buffer + 3, 2)) {
        sht41_on_error(SHT41_ERROR_CRC_INVALID);
    }
    local->rh = req->rx_buffer[3] << 8 | req->rx_buffer[4];

    err_count = 0;
    sht41_temp_rh_set_on_read = true;
    local->reading_expiration = make_timeout_time_ms(SHT41_INVALID_TIMEOUT_MS);
}

static void sht41_on_error(const sht41_error_code error_type) {
    err_count++;
    if (err_count >= ACCEPTABLE_ERR_COUNT) {
        if (error_cb) {
            error_cb(error_type);
        }
    }
}

// called by the i2c
static void on_async_i2c_error(__unused const struct async_i2c_request *req, __unused uint32_t error_code) {
    sht41_on_error(SHT41_ERROR_I2C_COMPLAINT);
}

void sht41_init(sht41_error_cb board_error_cb, uint8_t board, uint8_t addr) {
    error_cb = board_error_cb;
    last_reading.reading_expiration = get_absolute_time();
    last_reading.msg_in_progress = false;
    // wr_req.timeout = nil_time;
    // rd_req.timeout = nil_time;

    /*Initializing write_req and read_req inside last_reading*/
    last_reading.write_req.i2c_num = board;
    last_reading.write_req.address = addr;
    last_reading.write_req.nostop = false;
    last_reading.write_req.tx_buffer = write_cmd;
    last_reading.write_req.rx_buffer = NULL;
    last_reading.write_req.bytes_to_send = 1;
    last_reading.write_req.bytes_to_receive = 0;
    last_reading.write_req.completed_callback = &sht41_on_write;
    last_reading.write_req.failed_callback = &on_async_i2c_error;
    last_reading.write_req.next_req_on_success = NULL;
    last_reading.write_req.timeout = nil_time;
    last_reading.write_req.user_data = NULL;

    last_reading.read_req.i2c_num = board;
    last_reading.read_req.address = addr;
    last_reading.read_req.nostop = false;
    last_reading.read_req.tx_buffer = NULL;
    last_reading.read_req.rx_buffer = rx_buf;
    last_reading.read_req.bytes_to_send = 0;
    last_reading.read_req.bytes_to_receive = 6;
    last_reading.read_req.completed_callback = &sht41_on_read;
    last_reading.read_req.failed_callback = &on_async_i2c_error;
    last_reading.read_req.next_req_on_success = NULL;
    last_reading.read_req.timeout = nil_time;
    last_reading.read_req.user_data = NULL;

    if (add_alarm_in_ms(SHT41_POLL_TIME_MS, &sht41_start_write_req, (void *) &last_reading, true) < 0) {
        sht41_on_error(SHT41_ERROR_TIMER_SCHEDULE_FULL);
    }
}

// User function called by the board to get data

bool sht41_is_valid(void) {
    return !time_reached(last_reading.reading_expiration);
}

float sht41_read_temp(void) {
    return -45.0 + 175.0 * last_reading.temp / (65536.0 - 1.0);
}

float sht41_read_rh(void) {
    return -6.0 + 125.0 * last_reading.rh / (65536.0 - 1.0);
}
