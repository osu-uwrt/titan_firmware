#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/stdio_usb.h"

#include "pico_can_transport.h"
#include "basic_logger/logging.h"
#include "build_version.h"

#include "drivers/async_i2c.h"
#include "drivers/safety.h"
#include "hw/actuator.h"
#include "hw/balancer_adc.h"
#include "hw/depth_sensor.h"
#include "hw/dio.h"
#include "hw/dshot.h"
#include "hw/esc_adc.h"
#include "hw/esc_pwm.h"
#include "hw/bmp280_temp.h"
#include "tasks/cooling.h"
#include "tasks/lowbatt.h"
#include "tasks/ros.h"

#undef LOGGING_UNIT_NAME
#define LOGGING_UNIT_NAME "copro_main"
#undef LOGGING_UNIT_LOCAL_LEVEL
#define LOGGING_UNIT_LOCAL_LEVEL LEVEL_DEBUG

int main()
{
    // Immediate start code
    stdio_init_all();
    dual_usb_init();

    LOG_INFO(FULL_BUILD_TAG);
    sleep_ms(1000);

    // Safety must be the first item to initialize
    safety_setup();

    // Initialize sensor hardware
    dio_init();
    //bool got_temp_sensor = bmp280_temp_init();

    //async_i2c_init(200000, 5);
    //if (got_temp_sensor) bmp280_temp_start_reading();

    //depth_init();
    //actuator_init();
    //balancer_adc_init();
    //esc_adc_init();

    // Wait for ROS
    //uint8_t xavier_ip[] = {192, 168, 1, 23};
    //uint16_t xavier_port = 8888;
    //pico_eth_transport_init(0, *((uint32_t*)(&xavier_ip)), xavier_port);

    pico_can_transport_init(0,              // Take PIO 0
                            1000000,        // Set can bitrate to 1 Mbit
                            1,              // Set Client ID (no network defined, just taking 1)
                            ETH_MOSI_PIN,   // CAN RX Pin
                            ETH_MISO_PIN,   // CAN TX Pin
                            -1);            // No termination detection pin

    ros_wait_for_connection();
    ros_start(ROBOT_NAMESPACE);
    LOG_INFO("Connected to ROS");

    // Initialize safety-sensitive hardware
    safety_init();
// #if HW_USE_DSHOT
//     dshot_init();
// #endif
#if HW_USE_PWM
    esc_pwm_init();
#endif

    // Initialize any other tasks
    //cooling_init();
    //lowbatt_init();

    // Main Run Loop
    while (true)
    {
        safety_tick();
        ros_spin_ms(30);
        //cooling_tick();
        //lowbatt_tick();
    }
    return 0;
}
