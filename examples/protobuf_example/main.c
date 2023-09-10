#include "protos.pb.h"

#include "hardware/watchdog.h"
#include "pico/stdlib.h"
#include "titan/version.h"

#include <stdio.h>

const uint LED_PIN = PICO_DEFAULT_LED_PIN;

int main() {
    watchdog_enable(5000, true);
    stdio_init_all();
    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);
    printf("%s\n", FULL_BUILD_TAG);

    bool value = true;

    riptide_msgs2_FirmwareStatus firmware_status;
    firmware_status.client_id = 4;

    while (true) {
        watchdog_update();
        printf("Hello World!\n");

        gpio_put(LED_PIN, value);
        value = !value;

        sleep_ms(1000);
    }
    return 0;
}
