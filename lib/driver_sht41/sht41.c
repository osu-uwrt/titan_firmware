#include "driver/sht41.h"

#include "driver/async_i2c.h"

#include <stdio.h>

#define SHT41_WRITE_CMD 0xFD
#define SHT41_READ_LENGTH 6

// time needed for sht41 driver to complete sampling temperature and humidity
#define SHT41_SAMPLING_TIME_MS 15

// acceptable error count before calling back to board
#define SHT41_ACCEPTABLE_ERR_COUNT 3

// i2c address for sht41 driver
#define SHT41_I2C_ADDR 0x44

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

static struct sht41_data {
    uint16_t temp;
    uint16_t rh;
    absolute_time_t reading_expiration;
    struct async_i2c_request i2c_req;
    bool msg_in_progress;
    uint8_t rx_buf[SHT41_READ_LENGTH];
} last_reading = { 0 };

static sht41_error_cb error_cb;

static uint8_t err_count = 0;

static int64_t sht41_start_write_req(__unused alarm_id_t id, void *user_data) {
    struct sht41_data *local = (struct sht41_data *) user_data;

    local->i2c_req.tx_buffer = write_cmd;
    local->i2c_req.rx_buffer = NULL;
    local->i2c_req.bytes_to_send = sizeof(write_cmd);
    local->i2c_req.bytes_to_receive = 0;
    local->i2c_req.completed_callback = &sht41_on_write;

    async_i2c_enqueue(&local->i2c_req, &local->msg_in_progress);

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
    struct sht41_data *local = (struct sht41_data *) user_data;

    local->i2c_req.tx_buffer = NULL;
    local->i2c_req.rx_buffer = local->rx_buf;
    local->i2c_req.bytes_to_send = 0;
    local->i2c_req.bytes_to_receive = SHT41_READ_LENGTH;
    local->i2c_req.completed_callback = &sht41_on_read;

    async_i2c_enqueue(&local->i2c_req, &local->msg_in_progress);
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
    if (err_count >= SHT41_ACCEPTABLE_ERR_COUNT) {
        if (error_cb) {
            error_cb(error_type);
        }
    }
}

// called by the i2c
static void on_async_i2c_error(__unused const struct async_i2c_request *req, __unused uint32_t error_code) {
    sht41_on_error(SHT41_ERROR_I2C_COMPLAINT);
}

static void initialize_sht41_data(struct sht41_data *data, uint8_t i2c_bus_num) {
    data->reading_expiration = nil_time;
    data->msg_in_progress = false;

    /* Initializing i2c_req inside data */
    data->i2c_req.i2c_num = i2c_bus_num;
    data->i2c_req.address = SHT41_I2C_ADDR;
    data->i2c_req.nostop = false;
    data->i2c_req.tx_buffer = write_cmd;
    data->i2c_req.rx_buffer = NULL;
    data->i2c_req.bytes_to_send = sizeof(write_cmd);
    data->i2c_req.bytes_to_receive = 0;
    data->i2c_req.completed_callback = &sht41_on_write;
    data->i2c_req.failed_callback = &on_async_i2c_error;
    data->i2c_req.next_req_on_success = NULL;
    data->i2c_req.timeout = nil_time;
    data->i2c_req.user_data = data;
}

void sht41_init(sht41_error_cb board_error_cb, uint8_t i2c_bus_num) {
    error_cb = board_error_cb;

    initialize_sht41_data(&last_reading, i2c_bus_num);

    if (add_alarm_in_ms(SHT41_POLL_TIME_MS, &sht41_start_write_req, (void *) &last_reading, true) < 0) {
        err_count = SHT41_ACCEPTABLE_ERR_COUNT - 2;
        sht41_on_error(SHT41_ERROR_INITIAL_TIMER_SCHEDULE_FULL);
    }
}

// User function called by the board to get data

bool sht41_is_valid(void) {
    return !time_reached(last_reading.reading_expiration);
}

float sht41_read_temp(void) {
    return -45.0 + 175.0 * (float) (last_reading.temp) / (65536.0 - 1.0);
}

float sht41_read_rh(void) {
    return -6.0 + 125.0 * (float) (last_reading.rh) / (65536.0 - 1.0);
}
