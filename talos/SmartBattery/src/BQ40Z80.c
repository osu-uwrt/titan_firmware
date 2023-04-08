#include "BQ40Z80.h"
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/i2c.h"

#include <stdio.h>


uint8_t BQ_LEDS[3] = {LED_R_PIN, LED_Y_PIN, LED_G_PIN};

bool reserved_addr(uint8_t addr) {
    return (addr & 0x78) == 0 || (addr & 0x78) == 0x78;
}

uint8_t BQ_init() {
    uint8_t retries = 0;
    uint8_t data[2];
    data[0] = 0x1C;
    data[1] = 0x00;
    // Init the wake pin, active high
    gpio_init(BMS_WAKE_PIN);
    gpio_set_dir(BMS_WAKE_PIN, GPIO_OUT);
    gpio_put(BMS_WAKE_PIN, 0);

    for(uint8_t led = 0; led < 3; led++) {
        gpio_init(BQ_LEDS[led]);
        gpio_set_dir(BQ_LEDS[led], GPIO_OUT);
        gpio_put(BQ_LEDS[led], 0);
    }

    // init I2c peripheral on the pins
    i2c_init(i2c1, 100000);
    gpio_set_function(BMS_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(BMS_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(BMS_SDA_PIN);
    gpio_pull_up(BMS_SCL_PIN);


    // Start the I2C to the chip
    while((data[1] == 0x00 || data[1] == 0xFF) && retries < 3) {
        i2c_write_blocking(i2c1, BQ_ADDR, data, 1, false);
        i2c_read_blocking(i2c1, BQ_ADDR, data, 2, false);
        // Check for valid data
        if(data[1] == 0x00 || data[1] == 0xFF) {
            gpio_put(BMS_WAKE_PIN, 1);
            sleep_ms(1000);
            gpio_put(BMS_WAKE_PIN, 0);
            retries++;
        }
    }
    return retries;
}

void BQ_fault() {
    while(1) {
        gpio_put(BQ_LEDS[0], 1);
        sleep_ms(50);
        gpio_put(BQ_LEDS[0], 0);
    }
}