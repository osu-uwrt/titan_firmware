#include "pico/time.h"

#include "drivers/safety.h"
#include "hw/balancer_adc.h"
#include "hw/dio.h"
#include "tasks/cooling.h"

bool cooling_initialized;
int cooling_threshold = 40;

static bool cooling_enabled = false;

void cooling_tick(void) {
    hard_assert_if(LIFETIME_CHECK, !cooling_initialized);

    if (balancer_adc_initialized && !balancer_adc_readng_stale) {
        safety_lower_fault(FAULT_COOLING_STALE);

        bool enabled = balancer_adc_get_temperature() >= cooling_threshold;
        dio_set_peltier_power(enabled);
        cooling_enabled = enabled;
    } else {
        safety_raise_fault(FAULT_COOLING_STALE);
        dio_set_peltier_power(false);
        cooling_enabled = false;
    }
}

bool cooling_get_active(void) {
    return cooling_enabled;
}

void cooling_init(void) {
    hard_assert_if(LIFETIME_CHECK, cooling_initialized);
    cooling_initialized = true;
}