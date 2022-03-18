#include <stdio.h>
#include "pico/binary_info.h"
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/adc.h"

#include "drivers/safety.h"
#include "hw/dio.h"
#include "hw/dshot.h"
#include "hw/esc_pwm.h"

#undef LOGGING_UNIT_NAME
#define LOGGING_UNIT_NAME "dio"

bool dio_initialized = false;


// Pending fault led state in case dio hasn't been initialized when fault is raised
static bool pending_fault_led_state = false;

void dio_set_fault_led(bool on) {
    // Note: This method is called with interrupts disabled to prevent race conditions.

    if (dio_initialized) {
        gpio_put(FAULT_LED_PIN, on);
    } else {
        pending_fault_led_state = on;
    }
}


/**
 * @brief Reads the state of the kill switch
 * 
 * @return true If the kill switch is asserting kill
 * @return false If the kill switch is in enable mode
 */
static inline bool dio_get_kill_switch(void){
    return gpio_get(KILL_SWITCH_PIN);
}

/**
 * @brief The GPIO Interrupt Handler to handle all interrupts from the GPIO block
 * 
 * @param gpio Pin that caused the interrupt
 * @param events Events contained in the interrupt
 */
static void dio_gpio_callback(uint gpio, __unused uint32_t events) {
    if (gpio == KILL_SWITCH_PIN) {
        bool kill_switch_state = dio_get_kill_switch();
        safety_kill_switch_update(riptide_msgs2__msg__KillSwitchReport__KILL_SWITCH_PHYSICAL, kill_switch_state, false);

#if HW_USE_DSHOT
        dshot_notify_physical_kill_switch_change(kill_switch_state);
#endif
    }
}

bool dio_get_aux_switch(void) {
    hard_assert_if(LIFETIME_CHECK, !dio_initialized);

    return !gpio_get(AUX_SWITCH_PIN);
}

/**
 * @brief Callback for alarm to restore power to a rail
 * This is needed if the computer is on the power rail and would not be able to re-enable itself
 * 
 * @param id The alarm id
 * @param user_data The pin to toggle (stored directly in the 4-byte value)
 * @return int64_t If to reschedule
 */
static int64_t dio_power_restore_cb(__unused alarm_id_t id, void *user_data) {
    uint gpio_pin = (uint) user_data;
    gpio_put(gpio_pin, true);

    return 0;
}

#ifdef REG_12_CTRL_PIN
void dio_toggle_twelve_power(void) {
    hard_assert_if(LIFETIME_CHECK, !dio_initialized);

    gpio_put(REG_12_CTRL_PIN, false);
    hard_assert(add_alarm_in_ms(POWER_RAIL_TOGGLE_TIME_MS, &dio_power_restore_cb, (void*)REG_12_CTRL_PIN, true) > 0);
}
#endif

void dio_toggle_five_power(void) {
    hard_assert_if(LIFETIME_CHECK, !dio_initialized);

    gpio_put(REG_5_CTRL_PIN, false);
    hard_assert(add_alarm_in_ms(POWER_RAIL_TOGGLE_TIME_MS, &dio_power_restore_cb, (void*)REG_5_CTRL_PIN, true) > 0);
}

void dio_toggle_mobo_power(void) {
    hard_assert_if(LIFETIME_CHECK, !dio_initialized);

    gpio_put(MOBO_CTRL_PIN, false);
    hard_assert(add_alarm_in_ms(POWER_RAIL_TOGGLE_TIME_MS, &dio_power_restore_cb, (void*)MOBO_CTRL_PIN, true) > 0);
}

void dio_set_peltier_power(bool on) {
    hard_assert_if(LIFETIME_CHECK, !dio_initialized);

    gpio_put(PELTIER_CTRL_PIN, on);
}

double dio_get_battery_voltage_hack(void){
    hard_assert_if(LIFETIME_CHECK, !dio_initialized);
    const double conversion_factor = 3.3 / (double)(1 << 12) * (61.1+9.88) / 9.88;
    uint16_t result = adc_read();
    return conversion_factor*result;
}

void dio_init(void) {
    hard_assert_if(LIFETIME_CHECK, dio_initialized);

    // Fault LED
    bi_decl_if_func_used(bi_1pin_with_name(FAULT_LED_PIN, "Fault LED"));
    gpio_init(FAULT_LED_PIN);
    gpio_set_dir(FAULT_LED_PIN, GPIO_OUT);

    // Switches
    bi_decl_if_func_used(bi_1pin_with_name(KILL_SWITCH_PIN, "Kill Switch"));
    gpio_init(KILL_SWITCH_PIN);
    gpio_set_dir(KILL_SWITCH_PIN, GPIO_IN);
    gpio_pull_up(KILL_SWITCH_PIN);
    gpio_set_irq_enabled_with_callback(KILL_SWITCH_PIN, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true, &dio_gpio_callback);
    dio_gpio_callback(KILL_SWITCH_PIN, 0);

    bi_decl_if_func_used(bi_1pin_with_name(AUX_SWITCH_PIN, "Aux Switch"));
    gpio_init(AUX_SWITCH_PIN);
    gpio_set_dir(AUX_SWITCH_PIN, GPIO_IN);
    gpio_pull_up(AUX_SWITCH_PIN);

    // Power Control
    bi_decl_if_func_used(bi_1pin_with_name(PELTIER_CTRL_PIN, "Peltier Control"));
    gpio_init(PELTIER_CTRL_PIN);
    gpio_put(PELTIER_CTRL_PIN, false);
    gpio_set_dir(PELTIER_CTRL_PIN, true);
    
#ifdef REG_12_CTRL_PIN
    bi_decl_if_func_used(bi_1pin_with_name(REG_12_CTRL_PIN, "12V Regulator Conrol"));
    gpio_init(REG_12_CTRL_PIN);
    gpio_put(REG_12_CTRL_PIN, true);
    gpio_set_dir(REG_12_CTRL_PIN, true);
#endif

    bi_decl_if_func_used(bi_1pin_with_name(REG_5_CTRL_PIN, "5V Regulator Conrol"));
    gpio_init(REG_5_CTRL_PIN);
    gpio_put(REG_5_CTRL_PIN, true);
    gpio_set_dir(REG_5_CTRL_PIN, true);

    bi_decl_if_func_used(bi_1pin_with_name(MOBO_CTRL_PIN, "Mobo Power Conrol"));
    gpio_init(MOBO_CTRL_PIN);
    gpio_put(MOBO_CTRL_PIN, true);
    gpio_set_dir(MOBO_CTRL_PIN, true);

    adc_init();

    // Make sure GPIO is high-impedance, no pullups etc
    adc_gpio_init(LIGHTS_1_PIN);
    // Select ADC input 2 (GPIO28)
    adc_select_input(2);

    gpio_put(FAULT_LED_PIN, pending_fault_led_state);
    dio_initialized = true;
}