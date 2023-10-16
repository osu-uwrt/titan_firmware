#include "driver/sht41.h"

#include "driver/async_i2c.h"

#include <stdio.h>

#define SHT41_I2C_ADDR 0x44
#define SHT41_WRITE_CMD 0xFD
#define SHT41_READ_LENGTH 6

#define SHT41_SAMPLING_TIME_MS 15
#define SHT41_POLL_TIME_MS 500
#define SHT41_INVALID_TIMEOUT_MS 600

// fnc prototypes
static int64_t start_write_req(__unused alarm_id_t id, void *user_data);

static void on_write(const struct async_i2c_request *req);

static int64_t start_read_req(__unused alarm_id_t id, void *user_data);

static void on_read(const struct async_i2c_request *req);

// static void i2c_error_complaint(__unused const struct async_i2c_request *req, uint32_t error_code);

static void on_error(const struct async_i2c_request *req, uint32_t error_code);

// SHT41 data fields
static const uint8_t write_cmd[] = { SHT41_WRITE_CMD };
static uint8_t rx_buf[SHT41_READ_LENGTH];

// requests
static struct async_i2c_request wr_req = {
    .i2c_num = BOARD_I2C,
    .address = SHT41_I2C_ADDR,
    .nostop = false,
    .tx_buffer = write_cmd,
    .rx_buffer = NULL,
    .bytes_to_send = 1,
    .bytes_to_receive = 0,
    .completed_callback = &on_write,
    .failed_callback = &on_error,
    .next_req_on_success = NULL,
    .user_data = NULL  // TODO double check one this one
};

static struct async_i2c_request rd_req = {
    .i2c_num = BOARD_I2C,
    .address = SHT41_I2C_ADDR,
    .nostop = false,
    .tx_buffer = NULL,
    .rx_buffer = rx_buf,
    .bytes_to_send = 0,
    .bytes_to_receive = 6,
    .completed_callback = on_read,
    .failed_callback = &on_error,
    .next_req_on_success = NULL,
    .user_data = NULL  // FIXME double check on this one
};

static struct sht41_data {
    int16_t temp;
    int16_t rh;
    absolute_time_t reading_expiration;  // TODO nned to init in the init()
} last_reading = { 0 };

static bool msg_in_progress = false;

static sht41_error_cb error_cb;

static uint8_t err_count = 2;

static int64_t start_write_req(__unused alarm_id_t id, void *user_data) {
    printf("repeating timer alarm rcv\n");
    wr_req.user_data = user_data;
    async_i2c_enqueue(&wr_req, &msg_in_progress);
    return SHT41_POLL_TIME_MS * 1000;
}

static void on_write(const struct async_i2c_request *req) {
    // printf("begin on_write\n");  // FIXME debug only
    if (add_alarm_in_ms(SHT41_SAMPLING_TIME_MS, &start_read_req, req->user_data, true) < 0) {
        printf("Timer queue is full\n");  // FIXME debug only
        on_error(req, 0x0);
    }
    printf("10 ms alarm timer is added\n");  // FIXME debug only
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

static int64_t start_read_req(__unused alarm_id_t id, void *user_data) {
    rd_req.user_data = user_data;
    async_i2c_enqueue(&rd_req, &msg_in_progress);
    return 0;
}

static void on_read(const struct async_i2c_request *req) {
    printf("begin on_read()\n");  // FIXME debug only

    struct sht41_data *local = (struct sht41_data *) req->user_data;
    if (req->rx_buffer[2] != crc_check(req->rx_buffer, 2)) {
        printf("temp crc is invalid");  // FIXME debug only
        on_error(req, 0x5);
    }
    local->temp = req->rx_buffer[0] << 8 | req->rx_buffer[1];

    if (req->rx_buffer[5] != crc_check(req->rx_buffer + 3, 2)) {
        printf("rh crc is invalid");  // FIXME debug only
        on_error(req, 0x5);
    }
    local->rh = req->rx_buffer[3] << 8 | req->rx_buffer[4];

    err_count = 0;
    local->reading_expiration = make_timeout_time_ms(SHT41_INVALID_TIMEOUT_MS);
}

/* Deal with later */
// static void i2c_error_complaint(__unused const struct async_i2c_request *req, uint32_t error_code) {
//     on_error(SHT41_ERROR_I2C_COMPLAINT, error_code);
// }

// called by the i2c
static void on_error(const struct async_i2c_request *req, uint32_t error_code) {
    // process error
    // decode abort_data from i2c to get error_data
    // error callback to board: error_cb(error_data)
    // if (error_cb) {
    //     if (error == SHT41_ERROR_TIMER_SCHEDULE_FULL) {
    //         error_cb(error, error_code);
    //     }
    //     else {
    //         err_count++;
    //         if (err_count >= 3) {
    //             error_cb(error, error_code);
    //         }
    //     }
    // }

    err_count++;
    if (err_count >= 3) {
        if (error_cb) {
            error_cb(req, error_code);
        }
    }
}

void sht41_init(sht41_error_cb board_error_cb) {
    error_cb = board_error_cb;
    last_reading.reading_expiration = get_absolute_time();
    wr_req.timeout = nil_time;
    rd_req.timeout = nil_time;

    // Without using repeating alarm_timer_interrupts
    // wr_req.user_data = (void *) &last_reading;
    // printf("enqueuing wr_req\n");  // FIXME debug only
    // async_i2c_enqueue(&wr_req, &msg_in_progress);

    // Using repeating alarm_timer_interrupts
    if (add_alarm_in_ms(SHT41_POLL_TIME_MS, &start_write_req, (void *) &last_reading, true) < 0) {
        printf("Timer queue is full\n");  // FIXME debug only
        on_error(&wr_req, 0x0);
    }
    // error_cb if timer interrupt queue is full
}

// User function called by the board to get data

bool sht41_is_valid(void) {
    return !time_reached(last_reading.reading_expiration);  // valid if reading_expiration hasn't been reached
}

int16_t sht41_read_temp(void) {
    return last_reading.temp;
}

int16_t sht41_read_rh(void) {
    return last_reading.rh;
}
