#include <stdio.h>
#include "pico/stdlib.h"

#include "pico_eth_transport.h"
#include <types.h>

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

    // Eth init
    uint8_t xavier_ip[] = {192, 168, 1, 23};
    uint16_t xavier_port = 8888;
    pico_eth_transport_init(0, *((uint32_t*)(&xavier_ip)), xavier_port);

    printf("%s\n", FULL_BUILD_TAG);

    unsigned char count = 7;

    while (true) {
        printf("Hello World!\n");

        gpio_put(LED_RGB_R_PIN, count & 0x01);
        gpio_put(LED_RGB_G_PIN, count & 0x02);
        gpio_put(LED_RGB_B_PIN, count & 0x04);

        count = count > 0 ? 7 : count - 1;
        
        sleep_ms(1000);
    }
    return 0;
}