#include "solenoid.h"

#include "pico/stdlib.h"
#include "titan/logger.h"

#define SOLENOID_OPEN 1
#define SOLENOID_CLOSED 0

const int solenoid_pins[SOLENOID_COUNT] = {
    SOLENOID1_PIN,
    SOLENOID2_PIN,
    SOLENOID3_PIN,
};

void solenoid_init() {
    for (int i = 0; i < SOLENOID_COUNT; i++) {
        int pin = solenoid_pins[i];
        gpio_init(pin);
        gpio_set_dir(pin, GPIO_OUT);
        gpio_put(pin, SOLENOID_CLOSED);
    }
}

void solenoid_set(int number, bool open) {
    int idx = number - 1;
    if (idx >= 0 && number < SOLENOID_COUNT) {
        gpio_put(solenoid_pins[idx], open ? SOLENOID_OPEN : SOLENOID_CLOSED);
    }
    else {
        LOG_ERROR("cannot set solenoid: invalid number %d", number);
    }
}
