#include <math.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdint.h>

#include "async_i2c.h"
#include "lux_sensor.h"
#include "safety.h"

#define CMD_BYTE(cmd, word_protocol) ((1<<7) | (word_protocol ? 1 : 0) | (cmd))

#define LUX_I2C_BUS SENSOR_I2C_HW
#define LUX_I2C_ADDR 57

bool lux_initialized = false;

static bool in_transaction = false;

static const struct async_i2c_request read_ch0_req;
static int64_t lux_read_alarm_callback(alarm_id_t id, void *user_data) {
    async_i2c_enqueue(&read_ch0_req, &in_transaction);

    return 500000;
}

static void lux_read_failure(const struct async_i2c_request *req, uint32_t abort_data) {
    printf("Failed to read lux sensor (Abort Reg: 0x%x)\n", abort_data);
    safety_raise_fault(FAULT_LUX_ERROR);
}

static uint8_t ch0_data[2];
static uint8_t ch1_data[2];
static double ch0_raw = 0;
static double ch1_raw = 0;

static void lux_read_callback(const struct async_i2c_request *req) {
    ch0_raw = (ch0_data[1] << 8) + ch0_data[0];
    ch1_raw = (ch1_data[1] << 8) + ch1_data[0];
}

static const uint8_t read_ch1_cmd[] = {CMD_BYTE(0xE, true)};
static const struct async_i2c_request read_ch1_req = {\
    .i2c = SENSOR_I2C_HW,
    .address = LUX_I2C_ADDR,
    .nostop = false,
    .tx_buffer = read_ch1_cmd,
    .rx_buffer = ch1_data,
    .bytes_to_send = sizeof(read_ch1_cmd),
    .bytes_to_receive = sizeof(ch1_data),
    .completed_callback = &lux_read_callback,
    .failed_callback = &lux_read_failure,
    .next_req_on_success = NULL
};

static const uint8_t read_ch0_cmd[] = {CMD_BYTE(0xC, true)};
static const struct async_i2c_request read_ch0_req = {\
    .i2c = SENSOR_I2C_HW,
    .address = LUX_I2C_ADDR,
    .nostop = false,
    .tx_buffer = read_ch0_cmd,
    .rx_buffer = ch0_data,
    .bytes_to_send = sizeof(read_ch0_cmd),
    .bytes_to_receive = sizeof(ch0_data),
    .completed_callback = NULL,
    .failed_callback = &lux_read_failure,
    .next_req_on_success = &read_ch1_req
};

static void lux_init_failure(const struct async_i2c_request *req, uint32_t abort_data) {
    printf("Failed to initialize lux sensor (Abort Reg: 0x%x)\n", abort_data);
    safety_raise_fault(FAULT_LUX_ERROR);
}

static void lux_start_sensor_successful(const struct async_i2c_request *req) {
    lux_initialized = true;
    add_alarm_in_ms(400, &lux_read_alarm_callback, NULL, true);
}

static const uint8_t power_up_cmd[] = {CMD_BYTE(0, false), 3};
static const struct async_i2c_request power_up_req = {\
    .i2c = SENSOR_I2C_HW,
    .address = LUX_I2C_ADDR,
    .nostop = false,
    .tx_buffer = power_up_cmd,
    .rx_buffer = NULL,
    .bytes_to_send = sizeof(power_up_cmd),
    .bytes_to_receive = 0,
    .completed_callback = &lux_start_sensor_successful,
    .failed_callback = &lux_init_failure,
    .next_req_on_success = NULL
};

static uint8_t part_num = 0;
static void lux_part_read_successful(const struct async_i2c_request *req) {
    if (part_num == 0b01010000) {
        async_i2c_enqueue(&power_up_req, &in_transaction);
    } else {
        printf("Lux Sensor Part Number Mismatch! (0x%x returned)\n", part_num);
        safety_raise_fault(FAULT_LUX_ERROR);
    }
}

static const uint8_t get_part_cmd[] = {CMD_BYTE(0xA, false)};
static const struct async_i2c_request check_part_req = {\
    .i2c = SENSOR_I2C_HW,
    .address = LUX_I2C_ADDR,
    .nostop = false,
    .tx_buffer = get_part_cmd,
    .rx_buffer = &part_num,
    .bytes_to_send = sizeof(get_part_cmd),
    .bytes_to_receive = 1,
    .completed_callback = &lux_part_read_successful,
    .failed_callback = &lux_init_failure,
    .next_req_on_success = NULL
};

double lux_read(void) {
    if (ch0_raw == 0) {
        return 0;
    }

    double ch_ratio = ch1_raw / ch0_raw;
    double lux;

    if (0 < ch_ratio && ch_ratio <= 0.5) {
        lux = 0.0304 * ch0_raw - 0.062 * ch0_raw * pow(ch_ratio, 1.4);
    } else if (0.50 < ch_ratio && ch_ratio <= 0.61) {
        lux = 0.0224 * ch0_raw - 0.031 * ch1_raw;
    } else if (0.61 < ch_ratio && ch_ratio <= 0.80) {
        lux = 0.0128 * ch0_raw - 0.0153 * ch1_raw;
    } else if (0.80 < ch_ratio && ch_ratio <= 1.30) {
        lux = 0.00146 * ch0_raw - 0.00112 * ch1_raw;
    } else {
        lux = 0;
    }

    return lux;
}

void lux_init(void) {
    async_i2c_enqueue(&check_part_req, &in_transaction);
}