/**
 * @file bq25730.c
 * @brief Driver for the BQ25730 story.
 */

#include "bq25730.h"

#include "driver/async_i2c.h"
#include "titan/queue.h"

#include <stdio.h>
#include <string.h>

#define BQ25730_I2C_TOTAL_REGISTERS_SIZE 64
/**
 * @note All registers are 2 bytes except for Manufacturer ID and Device ID (which are 1 byte)
 */
#define BQ25730_I2C_CFG_REGISTER_SIZE 2

/**
 * The number of status registers to read during standby/charging.
 *
 * The registers are:
 * - Charger Status
 * - Prochot Status
 * - IIN_DPM
 * - ADC V-Bus / P-Sys
 * - ADC I BAT
 * - ADC IIN CMP IN
 * - ADC V-SYS / V-BAT
 */
#define BQ25730_STATUS_REGISTER_COUNT 7

/**
 * The amount of milliseconds to delay between polls of the status.
 */
#define BQ25730_STATUS_POLL_DELAY_MS 100

#define BQ25730_MSG_QUEUE_SIZE 32

#define BQ25730_MSG_MAX_SIZE 10 * BQ25730_I2C_CFG_REGISTER_SIZE

#define BQ25730_I2C_ADDRESS 0x6b

typedef enum bq25730_register {
    BQ25730_REG_CHARGE_OPTION_0 = 0x00,
    BQ25730_REG_CHARGE_CURRENT = 0x02,
    BQ25730_REG_OTG_VOLTAGE = 0x2E,
    BQ25730_REG_OTG_CURRENT = 0x2E,
    BQ25730_REG_INPUT_VOLTAGE = 0x0A,
    BQ25730_REG_VSYS_MIN = 0x0C,
    BQ25730_REG_IIN_HOST = 0x0E,
    BQ25730_REG_CHARGER_STATUS = 0x20,
    BQ25730_REG_PROCHOT_STATUS = 0x22,
    BQ25730_REG_IIN_DPM = 0x24,
    BQ25730_REG_ADCVBUSPSYS = 0x26,
    BQ25730_REG_ADCIBAT = 0x28,
    BQ25730_REG_ADCIINCMPIN = 0x2A,
    BQ25730_REG_ADCVSYSVBAT = 0x2C,
    BQ25730_REG_MANUFACTURER_ID = 0x2E,
    BQ25730_REG_DEVICE_ID = 0x2F,
    BQ25730_REG_CHARGE_OPTION_1 = 0x30,
    BQ25730_REG_CHARGE_OPTION_2 = 0x32,
    BQ25730_REG_CHARGE_OPTION_3 = 0x34,
    BQ25730_REG_PROCHOT_OPTION_0 = 0x36,
    BQ25730_REG_PROCHOT_OPTION_1 = 0x38,
    BQ25730_REG_ADCOPTION = 0x3A,
    BQ25730_REG_CHARGE_OPTION_4 = 0x3C,
    BQ25730_REG_VMIN_ACTIVE_PROTECTION = 0x3E
    // .. //
} bq25730_register_t;

struct bq25730_msg;

/**
 * @brief
 */
typedef void (*bq25730_on_msg_complete)(struct bq25730_msg *bq25730_msg);

typedef struct bq25730_msg {
    bool write;
    bq25730_register_t start_register;
    size_t register_count;
    uint8_t data[BQ25730_MSG_MAX_SIZE];
    bq25730_on_msg_complete on_complete;
} bq25730_msg_t;

static struct QUEUE_DEFINE(bq25730_msg_t, BQ25730_MSG_QUEUE_SIZE) msg_queue = {0};

static struct async_i2c_request i2c_req;

static uint8_t i2c_rx_buffer[BQ25730_I2C_TOTAL_REGISTERS_SIZE];
static uint8_t i2c_tx_buffer[BQ25730_I2C_TOTAL_REGISTERS_SIZE];

static bool i2c_request_in_progress = false;
static bq25730_state_t bq_state = BQ25730_STATE_DISCONNECTED;

struct bq25730_cfg_entry {
    bq25730_register_t reg;
    uint16_t data;
};

struct bq25730_cfg_entry cfg_entries[] = { { .reg = BQ25730_REG_ADCOPTION, .data = 0xFFFF },
                                           { .reg = BQ25730_REG_CHARGE_OPTION_0, .data = 0x4A03 } };
#define CFG_ENTRY_COUNT (sizeof(cfg_entries) / sizeof(*cfg_entries))

static bq25730_msg_t current_msg;

static void bq25730_send_next_msg_in_queue() {
    if(i2c_request_in_progress) {
        bq_state = BQ25730_STATE_ERROR;
        return;
    }

    bq25730_msg_t *msg = QUEUE_CUR_READ_ENTRY(&msg_queue);
    size_t registers_size = msg->register_count * BQ25730_I2C_CFG_REGISTER_SIZE;
    i2c_tx_buffer[0] = (uint8_t) msg->start_register;
    if(msg->write) {
        i2c_req.bytes_to_send = 1 + registers_size;
        i2c_req.bytes_to_receive = 0;
        memcpy(i2c_tx_buffer + 1, msg->data, registers_size);
    } else {
        i2c_req.bytes_to_send = 1;
        i2c_req.bytes_to_receive = registers_size;
    }
    i2c_req.timeout = make_timeout_time_ms(1000);
    current_msg = *msg;
    QUEUE_MARK_READ_DONE(&msg_queue);

    if(!async_i2c_enqueue(&i2c_req, &i2c_request_in_progress)){
        bq_state = BQ25730_STATE_ERROR;
    }
}

static void bq25730_on_i2c_req_complete(const struct async_i2c_request *req) {
    (void) req;

    if(current_msg.on_complete){
        current_msg.on_complete(&current_msg);
    }
    if(!QUEUE_EMPTY(&msg_queue)) {
        bq25730_send_next_msg_in_queue();
    }
}

static void bq25730_read_data(bq25730_register_t start_register, size_t register_count, bq25730_on_msg_complete on_read) {
    if(QUEUE_FULL(&msg_queue)) {
        bq_state = BQ25730_STATE_ERROR;
        return;
    }

    bq25730_msg_t *msg = QUEUE_CUR_WRITE_ENTRY(&msg_queue);
    msg->write = false;
    msg->start_register = start_register;
    msg->register_count = register_count;
    msg->on_complete = on_read;
    QUEUE_MARK_WRITE_DONE(&msg_queue);

    if(!i2c_request_in_progress) {
        bq25730_send_next_msg_in_queue();
    }
}

static void bq25730_write_data(bq25730_register_t start_register, uint8_t *data, size_t register_count, bq25730_on_msg_complete on_write) {
    if(QUEUE_FULL(&msg_queue) || register_count * BQ25730_I2C_CFG_REGISTER_SIZE > BQ25730_MSG_MAX_SIZE) {
        bq_state = BQ25730_STATE_ERROR;
        return;
    }

    bq25730_msg_t *msg = QUEUE_CUR_WRITE_ENTRY(&msg_queue);
    msg->write = true;
    msg->start_register = start_register;
    msg->register_count = register_count;
    msg->on_complete = on_write;
    memcpy(msg->data, data, register_count * 2);
    QUEUE_MARK_WRITE_DONE(&msg_queue);

    if(!i2c_request_in_progress) {
        bq25730_send_next_msg_in_queue();
    }
}

static void bq25730_on_i2c_failure(const struct async_i2c_request *req, uint32_t error) {
    (void) req;

    printf("Error: %lu\n", error);
    bq_state = BQ25730_STATE_ERROR;
}

static int64_t bq25730_poll_status();

static void on_status_polled(struct bq25730_msg *bq25730_msg) {
    add_alarm_in_ms(100, bq25730_poll_status, NULL, true);
    float result = (float) i2c_rx_buffer[0];
    result *= 0.073;
    result += 7.04;
    printf("voltage: %f\n", result);
}

/**
 * Starts a multi-read request of the BQ25730 status registers.
 *
 * @param on_status_polled a callback to handle the polled data.
 * @note This does not actually parse the data. To update the status,
 * you must called `bq25730_update_status_from_rx_buf` inside your callback.
 */
static int64_t bq25730_poll_status() {
    bq25730_read_data(BQ25730_REG_ADCVSYSVBAT, 1,on_status_polled);
    return 0;
}

/**
 * Parses the status from the registers read. This should be called
 * after the request from bq25730_poll_status is complete.
 */
// static void bq25730_update_status_from_rx_buf(bq25730_msg_t msg) {

// }

// ================
// Charging Setup
// ================

bq25730_status_t bq25730_start_charging() {
    if(bq_state != BQ25730_STATE_STANDBY) {
        return BQ25730_ERR;
    }

    bq_state = BQ25730_STATE_CHARGING_SETUP;
    return BQ25730_OK;
}

// ====================
// Standby State
// =====================

static void bq25730_enter_standby() {
    bq_state = BQ25730_STATE_STANDBY;
    bq25730_poll_status();
}

// ====================
// Configuration State
// =====================
static void bq25730_start_configuring() {
    bq_state = BQ25730_STATE_CONFIGURING;
    for(size_t i = 0; i < CFG_ENTRY_COUNT; i++) {
        struct bq25730_cfg_entry *entry = &cfg_entries[i];
        // no callback, except for last config entry, which switches you to standby state
        bq25730_on_msg_complete on_complete = i == CFG_ENTRY_COUNT - 1 ? bq25730_enter_standby : NULL;
        bq25730_write_data(entry->reg, (uint8_t*) &entry->data, 1, on_complete);
    }
}

void bq25730_init(unsigned int busNum) {
    i2c_req.i2c_num = busNum;
    i2c_req.address = BQ25730_I2C_ADDRESS;
    i2c_req.nostop = false;
    i2c_req.tx_buffer = i2c_tx_buffer;
    i2c_req.rx_buffer = i2c_rx_buffer;
    i2c_req.bytes_to_receive = 0;
    i2c_req.bytes_to_send = 0;
    i2c_req.completed_callback = bq25730_on_i2c_req_complete;
    i2c_req.failed_callback = bq25730_on_i2c_failure;
    i2c_req.next_req_on_success = NULL;

    bq25730_start_configuring();
}

bq25730_status_t bq25730_wait_until_configured() {
    return BQ25730_OK;
}

bq25730_state_t bq25730_get_state() {
    return bq_state;
}

const char *bq25730_state_to_string(bq25730_state_t state) {
    switch(state){
        case BQ25730_STATE_DISCONNECTED:
            return "disconnected";
        case BQ25730_STATE_CONFIGURING:
            return "configuring";
        case BQ25730_STATE_STANDBY:
            return "standby";
        case BQ25730_STATE_CHARGING:
            return "charging";
        case BQ25730_STATE_CHARGING_SETUP:
            return "setup charging";
        case BQ25730_STATE_ERROR:
            return "error";
        default:
            return "unknown";
    }
}

// static void bq25730_on_i2c_req_complete(const struct async_i2c_request *req) {
//     printf("Result: %i\n", i2c_rx_buffer[0]);
//     float result = (float) i2c_rx_buffer[0];
//     result *= 0.073;
//     result += 7.04;
//     printf("voltage: %f\n", result);
// }

// void bq25730_start_read_manufacturer_id() {
//     bq25730_read_data(BQ25730_REG_MANUFACTURER_ID);
// }

// void bq25730_start_read_system_voltage() {
//     bq25730_read_data(BQ25730_REG_ADCVSYSVBAT);
// }

// void bq25730_start_write_enable_low_power_mode() {
//     uint8_t data[] = { 0x4A, 0x03 };
//     bq25730_write_data(BQ25730_REG_CHARGE_OPTION_0, data, 2);
// }

// void bq25730_start_write_ADC_option() {
//     uint8_t data[] = { 0xFF, 0xFF };
//     bq25730_write_data(BQ25730_REG_ADCOPTION, data, 2);
// }

// void bq25730_start_read_input_voltage() {
//     bq25730_read_data(BQ25730_REG_INPUT_VOLTAGE);
// }
