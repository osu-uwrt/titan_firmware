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
#include "hw/esc_pwm.h"
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
    ros_start(ROBOT_NAMESPACE);
    printf("Connected to ROS\n");

    // Initialize safety-sensitive hardware
    safety_init();
#if HW_USE_DSHOT
    dshot_init();
#endif
#if HW_USE_PWM
    esc_pwm_init();
#endif

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