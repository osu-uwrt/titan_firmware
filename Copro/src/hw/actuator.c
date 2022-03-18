#include <stdio.h>
#include "drivers/async_i2c.h"

#include "basic_logging/logging.h"

#undef LOGGING_UNIT_NAME
#define LOGGING_UNIT_NAME "actuator_interface"

static void test_request_done(__unused const struct async_i2c_request * req) {
    LOG_INFO("Request succesfully sent");
}

static void test_request_failed(__unused const struct async_i2c_request * req, uint32_t abort_data){
    LOG_ERROR("Failed to send request: Abort Data 0x%x", abort_data);
}

static uint8_t test_data[4] = {0xFE, 0xED, 0xBE, 0xEF};

static bool request_in_progress = false;
static const struct async_i2c_request test_req = {
    .i2c = SENSOR_I2C_HW,
    .address = 0x3A,
    .nostop = false,
    .tx_buffer = test_data,
    .rx_buffer = NULL,
    .bytes_to_send = sizeof(test_data),
    .bytes_to_receive = 0,
    .completed_callback = test_request_done,
    .failed_callback = test_request_failed,
    .next_req_on_success = NULL,
    .user_data = NULL
};

void actuator_test(void){
    async_i2c_enqueue(&test_req, &request_in_progress);
}