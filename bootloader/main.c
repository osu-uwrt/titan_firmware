#include "hardware/gpio.h"
#include "hardware/timer.h"

int main(void) {
    gpio_init(PICO_DEFAULT_LED_PIN);
    gpio_put(PICO_DEFAULT_LED_PIN, 0);
    gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);

    while(1) {
        gpio_put(PICO_DEFAULT_LED_PIN, 0);
        busy_wait_ms(500);
        gpio_put(PICO_DEFAULT_LED_PIN, 1);
        busy_wait_ms(500);
    }
}