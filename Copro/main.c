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

int main()
{
    serial_init_early();
    printf("%s\n", FULL_BUILD_TAG);
    sleep_ms(1000);
    safety_setup();

    dio_init();
    async_i2c_init(200000, 10);
    lux_init();

    pico_serial_transport_init();
    ros_wait_for_connection();

    safety_init();
    dshot_init();
    ros_start("tempest");
    printf("Connected to ROS\n");

    while (true)
    {
        ros_spin_ms(30);
        safety_tick();
    }
    return 0;
}
