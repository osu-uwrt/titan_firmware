#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/stdio_usb.h"
#include "pico_uart_transports.h"
#include "hardware/watchdog.h"

#include "ros.h"
#include "basic_logger/logging.h"
#include "build_version.h"

#include "actuators/claw.h"
#include "actuators/dropper.h"
#include "actuators/torpedo.h"
#include "drivers/safety.h"

#undef LOGGING_UNIT_NAME
#define LOGGING_UNIT_NAME "actuator_main"

#define USE_POWER_LED 0

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

    pico_serial_transport_init();
    ros_wait_for_connection();

    claw_initialize();
    dropper_initialize();
    torpedo_initialize();

    ros_start("tempest");

    safety_init();
    safety_kill_switch_update(0, false, false);

    while(true) {
        ros_spin_ms(30);
        safety_tick();
    }

    return 0;
}