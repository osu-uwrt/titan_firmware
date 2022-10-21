#include <stdio.h>
#include "pico/stdlib.h"

#include "pico_uart_transports.h"
#include "basic_logger/logging.h"
#include "build_version.h"

#include "drivers/async_i2c.h"
#include "drivers/safety.h"
#include "hw/depth_sensor.h"
#include "tasks/ros.h"

#undef LOGGING_UNIT_NAME
#define LOGGING_UNIT_NAME "depth_main"
#undef LOGGING_UNIT_LOCAL_LEVEL
#define LOGGING_UNIT_LOCAL_LEVEL LEVEL_DEBUG

int main()
{
    // Immediate start code
    gpio_init(FAULT_LED_PIN);
    gpio_set_dir(FAULT_LED_PIN, GPIO_OUT);

    serial_init_early();
    LOG_INFO(FULL_BUILD_TAG);
    sleep_ms(1000);

    // Safety must be the first item to initialize
    safety_setup();

    async_i2c_init(200000, 5);
    depth_init();

    // Wait for ROS
    pico_serial_transport_init();
    ros_wait_for_connection();
    ros_start(ROBOT_NAMESPACE);
    LOG_INFO("Connected to ROS");

    // Initialize safety-sensitive hardware
    safety_init();

    // Main Run Loop
    while (true)
    {
        safety_tick();
        ros_spin_ms(30);
    }
    return 0;
}
