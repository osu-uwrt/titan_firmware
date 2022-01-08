#include <stdio.h>
#include "pico/stdlib.h"

#include "pico_uart_transports.h"
#include "build_version.h"

#include "dio.h"
#include "dshot.h"
#include "ros.h"
#include "safety.h"
#include "async_i2c.h"
#include "lux_sensor.h"
#include "depth_sensor.h"
#include "balancer_adc.h"
#include "esc_adc.h"

int main()
{
    serial_init_early();
    printf("%s\n", FULL_BUILD_TAG);
    sleep_ms(1000);
    safety_setup();

    dio_init();
    async_i2c_init(200000, 10);
    depth_init();
    lux_init();
    balancer_adc_init();
    esc_adc_init();

    pico_serial_transport_init();
    ros_wait_for_connection();

    safety_init();
    dshot_init();
    ros_start("tempest");
    printf("Connected to ROS\n");

    printf("Balanced Voltage: %.2f V\n", balancer_adc_get_balanced_voltage());
    printf("Thruster 0 Current: %.2f A\n", esc_adc_get_thruster_current(0));

    while (true)
    {
        ros_spin_ms(30);
        safety_tick();
    }
    return 0;
}
