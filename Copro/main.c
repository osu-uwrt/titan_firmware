#include <stdio.h>
#include "pico/stdlib.h"

#include "pico_uart_transports.h"
#include "build_version.h"

#include "safety.h"
#include "ros.h"

const uint LED_PIN = 25;

int main()
{
    serial_init_early();
    printf("%s\n", FULL_BUILD_TAG);
    safety_setup();

    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);

    pico_serial_transport_init();
    ros_wait_for_connection();

    safety_init();
    ros_start("tempest");
    printf("Connected to ROS\n");
    gpio_put(LED_PIN, 1);

    while (true)
    {
        ros_spin_ms(30);
        safety_tick();
    }
    return 0;
}
