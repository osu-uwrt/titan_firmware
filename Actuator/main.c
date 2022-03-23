#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/stdio_usb.h"
#include "hardware/i2c.h"
#include "hardware/watchdog.h"

#include "actuator_i2c/interface.h"
#include "basic_logging/logging.h"
#include "build_version.h"

#include "async_i2c_target.h"

#undef LOGGING_UNIT_NAME
#define LOGGING_UNIT_NAME "actuator_main"

const uint LED_PIN = PICO_DEFAULT_LED_PIN;

int main() {
    stdio_init_all();
    dual_usb_init();
    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);
    gpio_put(LED_PIN, true);
    LOG_INFO("%s", FULL_BUILD_TAG);

    async_i2c_target_init(200000, ACTUATOR_I2C_ADDR);

    if (watchdog_enable_caused_reboot())
        LOG_ERROR("Watchdog Reset");
    watchdog_enable(3000, true);

    bool value = true;

    actuator_i2c_cmd_t my_cmd;
    actuator_i2c_response_t my_response;

    LOG_INFO("Waiting for data...");
    while (true) {
        if (async_i2c_target_get_next_command(&my_cmd)) {
            LOG_INFO("New Command: %d", my_cmd.cmd_id);

            size_t command_size = 2;
            size_t response_data_size = 0;
            switch (my_cmd.cmd_id) {
                case ACTUATOR_CMD_CLAW_TIMING:
                    LOG_INFO("Claw Timings Command - Open Time: %d ms - Close Time: %d ms", my_cmd.data.claw_timing.open_time_ms, my_cmd.data.claw_timing.close_time_ms);
                    command_size += ACTUATOR_CMD_CLAW_TIMING_LENGTH;

                    response_data_size = sizeof(my_response.data.result);
                    my_response.data.result = ACTUATOR_RESULT_SUCCESSFUL;
                    break;
                case ACTUATOR_CMD_TEST:
                    LOG_INFO("Test Command - Data 1: 0x%08x - Data 2: 0x%08x", my_cmd.data.test.data1, my_cmd.data.test.data2);
                    command_size += ACTUATOR_CMD_TEST_LENGTH;

                    response_data_size = sizeof(my_response.data.result);
                    my_response.data.result = ACTUATOR_RESULT_SUCCESSFUL;
                    break;
                case ACTUATOR_CMD_OPEN_CLAW:
                    LOG_INFO("Open Claw Command");
                default:
                    LOG_INFO("Unknown command!");
            }

            printf("Raw Command Data: ");
            uint8_t *raw_data = (uint8_t*)(&my_cmd);
            for (int i = 0; i < command_size; i++) {
                printf("%s0x%02x", (i ==0 ? "" : ", "), raw_data[i]);
            }
            printf("\n\n");

            size_t response_size = 0;
            if (response_data_size != 0) {
                response_size = ACTUATOR_BASE_RESPONSE_LENGTH + response_data_size;
            }

            assert(response_size == ACTUATOR_GET_RESPONSE_SIZE(my_cmd.cmd_id));
            async_i2c_target_finish_command(&my_response, response_size);

            LOG_INFO("Waiting for data...");
        }
        watchdog_update();
    }
    return 0;
}