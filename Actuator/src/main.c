#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/stdio_usb.h"
#include "hardware/i2c.h"
#include "hardware/watchdog.h"

#include "actuator_i2c/interface.h"
#include "basic_logger/logging.h"
#include "build_version.h"

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

    // TODO: Populate this
    status->claw_state = CLAW_STATE_UNITIALIZED;
    status->dropper1_state = DROPPER_STATE_UNITIALIZED;
    status->dropper2_state = DROPPER_STATE_UNITIALIZED;
    status->torpedo1_state = TORPEDO_STATE_UNITIALIZED;
    status->torpedo2_state = TORPEDO_STATE_UNITIALIZED;
}

const uint LED_PIN = PICO_DEFAULT_LED_PIN;

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
                    LOG_INFO("Opening Claw");
                    response.data.result = ACTUATOR_RESULT_FAILED;
                    response_size = ACTUATOR_RESULT_RESP_LENGTH;
                    break;
                case ACTUATOR_CMD_CLOSE_CLAW:
                    LOG_INFO("Closing Claw");
                    response.data.result = ACTUATOR_RESULT_FAILED;
                    response_size = ACTUATOR_RESULT_RESP_LENGTH;
                    safety_raise_fault(FAULT_CLAW_ERROR);
                    break;
                case ACTUATOR_CMD_CLAW_TIMING:
                    LOG_INFO("Setting claw timing (Open %d ms, Close %d ms)", cmd.data.claw_timing.open_time_ms, cmd.data.claw_timing.close_time_ms);
                    response.data.result = ACTUATOR_RESULT_FAILED;
                    response_size = ACTUATOR_RESULT_RESP_LENGTH;
                    break;
                case ACTUATOR_CMD_ARM_TORPEDO:
                    LOG_INFO("Arming Torpedo");
                    response.data.result = ACTUATOR_RESULT_FAILED;
                    response_size = ACTUATOR_RESULT_RESP_LENGTH;
                    break;
                case ACTUATOR_CMD_DISARM_TORPEDO:
                    LOG_INFO("Disarming Torpedo");
                    response.data.result = ACTUATOR_RESULT_FAILED;
                    response_size = ACTUATOR_RESULT_RESP_LENGTH;
                    break;
                case ACTUATOR_CMD_FIRE_TORPEDO:
                    LOG_INFO("Firing Torpedo %d", cmd.data.fire_torpedo.torpedo_num);
                    response.data.result = ACTUATOR_RESULT_FAILED;
                    response_size = ACTUATOR_RESULT_RESP_LENGTH;
                    break;
                case ACTUATOR_CMD_TORPEDO_TIMING:
                    LOG_INFO("Setting Torpedo Timings for torpedo %d (Timing type: %d, timing: %d us)", cmd.data.torpedo_timing.torpedo_num, cmd.data.torpedo_timing.timing_type, cmd.data.torpedo_timing.time_us);
                    response.data.result = ACTUATOR_RESULT_FAILED;
                    response_size = ACTUATOR_RESULT_RESP_LENGTH;
                    break;
                case ACTUATOR_CMD_DROP_MARKER:
                    LOG_INFO("Dropping Marker %d", cmd.data.drop_marker.marker_num);
                    response.data.result = ACTUATOR_RESULT_FAILED;
                    response_size = ACTUATOR_RESULT_RESP_LENGTH;
                    break;
                case ACTUATOR_CMD_CLEAR_DROPPER_STATUS:
                    LOG_INFO("Clearing Dropper Status");
                    response.data.result = ACTUATOR_RESULT_FAILED;
                    response_size = ACTUATOR_RESULT_RESP_LENGTH;
                    assert(false);
                    break;
                case ACTUATOR_CMD_MARKER_TIMING:
                    LOG_INFO("Setting marker timings to %d ms", cmd.data.marker_timing.active_time_ms);
                    response.data.result = ACTUATOR_RESULT_FAILED;
                    response_size = ACTUATOR_RESULT_RESP_LENGTH;
                    break;
                case ACTUATOR_CMD_KILL_SWITCH:
                    safety_kill_switch_update(KILL_SWITCH_I2C_MSG, cmd.data.kill_switch.asserting_kill, true);
                    response.data.result = ACTUATOR_RESULT_SUCCESSFUL;
                    response_size = ACTUATOR_RESULT_RESP_LENGTH;
                    break;
                case ACTUATOR_CMD_RESET_ACTUATORS:
                    LOG_INFO("Reboot command sent");
                    watchdog_reboot(0, 0, 0);
                    break;
                default:
                    LOG_WARN("Unknown command 0x%02x", cmd.cmd_id);
            }

            async_i2c_target_finish_command(&response, response_size);
        }
        safety_tick();
    }
    return 0;
}