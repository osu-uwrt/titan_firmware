#include <stdio.h>
#include "pico/stdlib.h"

#include "build_version.h"

#define PIN_RGB_R 29
#define PIN_RGB_G 28
#define PIN_RGB_B 27

int main() {
    stdio_init_all();

    // GPIO init
    gpio_init(PIN_RGB_R);
    gpio_set_dir(PIN_RGB_R, GPIO_OUT);
    gpio_init(PIN_RGB_G);
    gpio_set_dir(PIN_RGB_G, GPIO_OUT);
    gpio_init(PIN_RGB_B);
    gpio_set_dir(PIN_RGB_B, GPIO_OUT);


    printf("%s\n", FULL_BUILD_TAG);

    unsigned char count = 0;

    while (true) {
        printf("Hello World!\n");

        gpio_put(PIN_RGB_R, count & 0x01);
        gpio_put(PIN_RGB_G, count & 0x02);
        gpio_put(PIN_RGB_B, count & 0x04);

        count = count >= 8 ? 0 : count + 1;
        
        sleep_ms(1000);
    }
    return 0;
}