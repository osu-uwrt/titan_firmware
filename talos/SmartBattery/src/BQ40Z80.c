#include "BQ40Z80.h"
#include "pio_i2c.h"
#include "pico/stdlib.h"

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


    uint offset = pio_add_program(pio0, &i2c_program);
    i2c_program_init(pio0, 0, offset, BMS_SDA_PIN, BMS_SCL_PIN);

    printf("\nPIO I2C Bus Scan\n");
    printf("   0  1  2  3  4  5  6  7  8  9  A  B  C  D  E  F\n");

    for (int addr = 0; addr < (1 << 7); ++addr) {
        if (addr % 16 == 0) {
            printf("%02x ", addr);
        }
        // Perform a 0-byte read from the probe address. The read function
        // returns a negative result NAK'd any time other than the last data
        // byte. Skip over reserved addresses.
        int result;
        if (reserved_addr(addr))
            result = -1;
        else
            result = pio_i2c_read_blocking(pio0, 0, addr, NULL, 0);

        printf(result < 0 ? "." : "@");
        printf(addr % 16 == 15 ? "\n" : "  ");
    }
    printf("Done.\n");
    panic("BRRRRR");

    // Start Roberts Jank I2C implementation
    // while((data[1] == 0x00 || data[1] == 0xFF) && retries < 3) {
    //     pio_i2c_write_blocking(pio0, 0, BQ_ADDR, data, 1);
    //     pio_i2c_read_blocking(pio0, 0, BQ_ADDR, data, 2);
    //     // Check for valid data
    //     if(data[1] == 0x00 || data[1] == 0xFF) {
    //         gpio_put(BMS_WAKE_PIN, 1);
    //         sleep_ms(1000);
    //         gpio_put(BMS_WAKE_PIN, 0);
    //         retries++;
    //     }
    // }
    return retries;
}

void BQ_fault() {
    while(1) {
        gpio_put(BQ_LEDS[0], 1);
        sleep_ms(50);
        gpio_put(BQ_LEDS[0], 0);
    }
}