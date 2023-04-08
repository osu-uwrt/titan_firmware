#include <stdio.h>

#include "pico/stdlib.h"
#include "pio_i2c.h"

#include "basic_logger/logging.h"
#include "build_version.h"

// might be 0x0B 
#define BQ_ADDR 0x0B
#define PIO_SM 0

int main(){
    stdio_init_all();
    
    //init GPIO
    // Latch RP2040 power to on
    gpio_init(PWR_CTRL_PIN);
    gpio_set_dir(PWR_CTRL_PIN, GPIO_OUT);
    gpio_put(PWR_CTRL_PIN, 1);

    gpio_init(LED_R_PIN);
    gpio_set_dir(LED_R_PIN, GPIO_OUT);
    gpio_put(LED_R_PIN, 0);

    gpio_init(LED_G_PIN);
    gpio_set_dir(LED_G_PIN, GPIO_OUT);
    gpio_put(LED_G_PIN, 0);

    // Initialize stdio
    stdio_init_all();
    LOG_INFO("%s", FULL_BUILD_TAG);

    gpio_put(LED_G_PIN, 1);
    gpio_put(LED_R_PIN, 1);

    sleep_ms(1000);

    gpio_put(LED_G_PIN, 0);
    gpio_put(LED_R_PIN, 0);


    // init PIO I2C
    uint offset = pio_add_program(pio0, &i2c_program);
    i2c_program_init(pio0, PIO_SM, offset, BMS_SDA_PIN, BMS_SCL_PIN);

    //try to contact the chip
    while(true){
        // try to init the chip and read a serial#
        uint8_t data[5] = {0x22, 0x00};
        pio_i2c_write_blocking(pio0, 0, BQ_ADDR, data, 1);
        pio_i2c_read_blocking(pio0, 0, BQ_ADDR, data, 5);

        printf("%02x %02x %02x %02x %02x\n", data[0]&0xff, data[1]&0xff, data[2]&0xff, data[3]&0xff, data[4]&0xff);

        sleep_ms(1000);
    }

    return 0;
}