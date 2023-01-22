#include <stdio.h>
#include "pico/stdlib.h"

#include "build_version.h"

int main() {
    stdio_init_all();

    // GPIO init
    gpio_init(LED_RGB_R_PIN);
    gpio_set_dir(LED_RGB_R_PIN, GPIO_OUT);
    gpio_init(LED_RGB_G_PIN);
    gpio_set_dir(LED_RGB_G_PIN, GPIO_OUT);
    gpio_init(LED_RGB_B_PIN);
    gpio_set_dir(LED_RGB_B_PIN, GPIO_OUT);


    printf("%s\n", FULL_BUILD_TAG);

    unsigned char count = 0;

    while (true) {
        printf("Hello World!\n");

        gpio_put(LED_RGB_R_PIN, count & 0x01);
        gpio_put(LED_RGB_G_PIN, count & 0x02);
        gpio_put(LED_RGB_B_PIN, count & 0x04);

        count = count >= 8 ? 0 : count + 1;
        
        sleep_ms(1000);
    }
    return 0;
}