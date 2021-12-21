#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"

#include "dio.h"
#include "safety.h"


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
static void dio_gpio_callback(uint gpio, uint32_t events) {
    if (gpio == KILL_SWITCH_PIN) {
        bool kill_switch_state = dio_get_kill_switch();
        safety_kill_switch_update(KILL_SWITCH_ID_PHYSICAL, kill_switch_state, false);
    }
}


void dio_init(void) {
    if (dio_initialized) {
        panic("DIO Already Initialized");
    }

    // Fault LED
    gpio_init(FAULT_LED_PIN);
    gpio_set_dir(FAULT_LED_PIN, GPIO_OUT);
    gpio_put(FAULT_LED_PIN, pending_fault_led_state);

    // Kill Switch
    gpio_init(KILL_SWITCH_PIN);
    gpio_set_dir(KILL_SWITCH_PIN, GPIO_IN);
    gpio_pull_up(KILL_SWITCH_PIN);
    gpio_set_irq_enabled_with_callback(KILL_SWITCH_PIN, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true, &dio_gpio_callback);
    safety_kill_switch_update(KILL_SWITCH_ID_PHYSICAL, dio_get_kill_switch(), false);

    dio_initialized = true;
}