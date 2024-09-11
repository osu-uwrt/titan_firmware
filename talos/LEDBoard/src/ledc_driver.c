#include "ledc_driver.h"

#include "ledc_commands.h"

#include "hardware/adc.h"
#include "hardware/clocks.h"
#include "hardware/gpio.h"
#include "hardware/pwm.h"
#include "hardware/spi.h"
#include "hardware/timer.h"
#include "pico/binary_info.h"
#include "pico/stdlib.h"
#include "pico/time.h"
#include "titan/debug.h"

#define LED_UPDATE_INTERVAL_MS 50
#define CONTROLLER_WATCHDOG_PERIOD_MS 3

#define OPERATING_TEMPERATURE_MAX_C 50

repeating_timer_t controller_watchdog_timer;

void ledc_init() {
    init_spi_and_gpio();

    // LEDC defines set in ledc_commands.h
    for (uint controller = LEDC1; controller <= LEDC2; controller++) {
        for (uint buck = BUCK1; buck <= BUCK2; buck++) {
            // Sets LEDs to off by default using PWM dimming
            buck_set_control_mode(controller, buck, BUCK_PWM_DIMMING);
            sleep_ms(1);
        }

        controller_clear_watchdog_error(controller);
        controller_enable(controller);
    }

    add_repeating_timer_ms(CONTROLLER_WATCHDOG_PERIOD_MS, controller_satisfy_watchdog, NULL,
                           &controller_watchdog_timer);
}
