#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/stdio_usb.h"
#include "hardware/i2c.h"
#include "hardware/watchdog.h"

#include "actuator_i2c/interface.h"
#include "basic_logger/logging.h"
#include "build_version.h"

#include "actuators/claw.h"
#include "actuators/dropper.h"
#include "actuators/torpedo.h"
#include "drivers/async_i2c_target.h"
#include "drivers/safety.h"

#undef LOGGING_UNIT_NAME
#define LOGGING_UNIT_NAME "actuator_main"

static void populate_status_msg(struct actuator_i2c_status *status){
    // NOTE: The firmware version in the CMakeLists should match the expected firmware version in actuator_i2c/interface.h
    status->firmware_status.version_major = MAJOR_VERSION;
    status->firmware_status.version_minor = MINOR_VERSION;

    uint32_t faults = *fault_list;
    assert(faults < (1<<8));
    status->firmware_status.fault_list = faults;

    claw_populate_missing_timings(&status->firmware_status.missing_timings);
    dropper_populate_missing_timings(&status->firmware_status.missing_timings);
    torpedo_populate_missing_timings(&status->firmware_status.missing_timings);

    status->claw_state = claw_get_state();
    status->dropper1_state = dropper_get_state(1);
    status->dropper2_state = dropper_get_state(2);
    status->torpedo1_state = torpedo_get_state(1);
    status->torpedo2_state = torpedo_get_state(2);
}

const uint LED_PIN = BUILTIN_LED2_PIN;

int main() {
    stdio_init_all();
    dual_usb_init();
    LOG_INFO("%s", FULL_BUILD_TAG);

    safety_setup();

    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);
    gpio_put(LED_PIN, true);

    async_i2c_target_init(200000, ACTUATOR_I2C_ADDR);

    safety_init();

    claw_initialize();
    dropper_initialize();
    torpedo_initialize();

    actuator_i2c_cmd_t cmd;
    actuator_i2c_response_t response;

    while (true) {
        if (async_i2c_target_get_next_command(&cmd)) {
            LOG_DEBUG("Received Command: %d", cmd.cmd_id);

            size_t response_size = 0;
            switch (cmd.cmd_id) {
                case ACTUATOR_CMD_GET_STATUS:
                    populate_status_msg(&response.data.status);
                    response_size = ACTUATOR_STATUS_RESP_LENGTH;
                    break;
                case ACTUATOR_CMD_OPEN_CLAW:
                    response.data.result = claw_open();
                    response_size = ACTUATOR_RESULT_RESP_LENGTH;
                    break;
                case ACTUATOR_CMD_CLOSE_CLAW:
                    response.data.result = claw_close();
                    response_size = ACTUATOR_RESULT_RESP_LENGTH;
                    break;
                case ACTUATOR_CMD_CLAW_TIMING:
                    response.data.result = claw_set_timings(&cmd.data.claw_timing);
                    response_size = ACTUATOR_RESULT_RESP_LENGTH;
                    break;
                    
                case ACTUATOR_CMD_ARM_TORPEDO:
                    response.data.result = torpedo_arm();
                    response_size = ACTUATOR_RESULT_RESP_LENGTH;
                    break;
                case ACTUATOR_CMD_DISARM_TORPEDO:
                    response.data.result = torpedo_disarm();
                    response_size = ACTUATOR_RESULT_RESP_LENGTH;
                    break;
                case ACTUATOR_CMD_FIRE_TORPEDO:
                    response.data.result = torpedo_fire(&cmd.data.fire_torpedo);
                    response_size = ACTUATOR_RESULT_RESP_LENGTH;
                    break;
                case ACTUATOR_CMD_TORPEDO_TIMING:
                    response.data.result = torpedo_set_timings(&cmd.data.torpedo_timing);
                    response_size = ACTUATOR_RESULT_RESP_LENGTH;
                    break;

                case ACTUATOR_CMD_DROP_MARKER:
                    response.data.result = dropper_drop_marker(&cmd.data.drop_marker);
                    response_size = ACTUATOR_RESULT_RESP_LENGTH;
                    break;
                case ACTUATOR_CMD_CLEAR_DROPPER_STATUS:
                    response.data.result = dropper_clear_status();
                    response_size = ACTUATOR_RESULT_RESP_LENGTH;
                    break;
                case ACTUATOR_CMD_DROPPER_TIMING:
                    response.data.result = dropper_set_timings(&cmd.data.dropper_timing);
                    response_size = ACTUATOR_RESULT_RESP_LENGTH;
                    break;

                case ACTUATOR_CMD_KILL_SWITCH:
                    safety_kill_switch_update(KILL_SWITCH_I2C_MSG, cmd.data.kill_switch.asserting_kill, true);
                    response.data.result = ACTUATOR_RESULT_SUCCESSFUL;
                    response_size = ACTUATOR_RESULT_RESP_LENGTH;
                    break;

                case ACTUATOR_CMD_RESET_ACTUATORS:
                    LOG_INFO("Reboot command received");
                    watchdog_reboot(0, 0, 0);
                    break;

                default:
                    LOG_WARN("Unknown command 0x%02x", cmd.cmd_id);
                    safety_raise_fault(FAULT_I2C_PROTO_ERROR);
                    break;
            }

            async_i2c_target_finish_command(&response, response_size);
        }
        safety_tick();
    }
    return 0;
}