#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/stdio_usb.h"
#include "pico_uart_transports.h"
#include "hardware/i2c.h"
#include "hardware/watchdog.h"

#include "ros.h"
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

#define USE_POWER_LED 0

__unused static void populate_status_msg(struct actuator_i2c_status *status){
    // NOTE: The firmware version in the CMakeLists should match the expected firmware version in actuator_i2c/interface.h
    status->firmware_status.version_major = MAJOR_VERSION;
    status->firmware_status.version_minor = MINOR_VERSION;

    uint32_t faults = *fault_list_reg;
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

int main() {
    serial_init_early();
    LOG_INFO("%s", FULL_BUILD_TAG);

    safety_setup();

    #if USE_POWER_LED
    const uint LED_PIN = BUILTIN_LED2_PIN;
    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);
    gpio_put(LED_PIN, USE_POWER_LED);
    #endif

    //async_i2c_target_init(200000, ACTUATOR_I2C_ADDR);
    pico_serial_transport_init();
    ros_wait_for_connection();
    ros_start("tempest");

    safety_init();
    safety_kill_switch_update(0, false, false);

    claw_initialize();
    dropper_initialize();
    torpedo_initialize();

    while(true) {
        ros_spin_ms(30);
        safety_tick();
    }

    return 0;
}