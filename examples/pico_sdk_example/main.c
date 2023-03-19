#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/watchdog.h"

#include "build_version.h"

const uint LED_PIN = STATUS_LEDR_PIN;

int main() {
    watchdog_enable(5000, true);
    stdio_init_all();
    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);
    printf("%s\n", FULL_BUILD_TAG);

    bool value = true;

    while (true) {
        watchdog_update();
        printf("Hello World!\n");

        gpio_put(LED_PIN, value);
        value = !value;

        sleep_ms(1000);
    }
    return 0;
}
