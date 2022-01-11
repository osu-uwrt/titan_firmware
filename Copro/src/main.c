#include <stdio.h>
#include "pico/stdlib.h"

#include "pico_uart_transports.h"
#include "build_version.h"

#include "drivers/async_i2c.h"
#include "drivers/safety.h"
#include "hw/balancer_adc.h"
#include "hw/depth_sensor.h"
#include "hw/dio.h"
#include "hw/dshot.h"
#include "hw/esc_adc.h"
#include "tasks/cooling.h"
#include "tasks/lowbatt.h"
#include "tasks/ros.h"

int main()
{
    // Immediate start code
    serial_init_early();
    printf("%s\n", FULL_BUILD_TAG);
    sleep_ms(1000);

    // Safety must be the first item to initialize
    safety_setup();

    // Initialize sensor hardware
    dio_init();
    async_i2c_init(200000, 10);
    depth_init();
    balancer_adc_init();
    esc_adc_init();

    // Wait for ROS
    pico_serial_transport_init();
    ros_wait_for_connection();

    // Initialize safety-sensitive hardware
    safety_init();
    dshot_init();

    // Start ROS
    ros_start("tempest");
    printf("Connected to ROS\n");

    // Initialize any other tasks
    cooling_init();
    lowbatt_init();

    // Main Run Loop
    while (true)
    {
        safety_tick();
        ros_spin_ms(30);
        cooling_tick();
        lowbatt_tick();
    }
    return 0;
}
