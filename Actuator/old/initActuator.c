#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "hardware/watchdog.h"
#include "hardware/timer.h"
#include "hardware/clocks.h"
#include "hardware/gpio.h"
#include <string.h>
#include "hardware/pwm.h"
#include "hardware/irq.h"
#include "Actuator.h"

const uint8 HIGHPRIOR=0;
const bool ENABLE=1;
//Initization functions for differnt ports
void init_pwm(uint16 top){
    //PWM PGIO setup for old claw
    pwm_config cfg = pwm_get_default_config();
    pwm_config_set_wrap(&cfg, top);
    pwm_init(pwm_gpio_to_slice_num(PWM_CLAW), &cfg, true);
    gpio_set_function(PWM_CLAW, GPIO_FUNC_PWM);
    //gpio_set_function(PWM_DRIVER1, GPIO_FUNC_PWM);
    //gpio_set_function(PWM_DRIVER2, GPIO_FUNC_PWM);
    //Getting PWM slice number
    //uint slice_num1 = pwm_gpio_to_slice_num(PWM_DRIVER1);
    //uint slice_num2 = pwm_gpio_to_slice_num(PWM_DRIVER2);

    //PWM claw can be 400HZ max with 1500msec high and can base the duty cycle
    return;
}

void init_new_claw(){
    //These are not really PWM we only need a simple input output to controll the new claw
    gpio_set_function(PWM_DRIVER1, GPIO_FUNC_PIO0);
    gpio_set_dir(PWM_DRIVER1, OUT);
    gpio_put(PWM_DRIVER1,false);
    gpio_set_function(PWM_DRIVER2, GPIO_FUNC_PIO0);
    gpio_set_dir(PWM_DRIVER2, OUT);
    gpio_put(PWM_DRIVER2,false);
    return;
}
void init_dropper(){
    gpio_set_function(DROPPER1_GATE, GPIO_FUNC_PIO0);
    gpio_set_dir(DROPPER1_GATE, OUT);
    gpio_set_function(DROPPER2_GATE, GPIO_FUNC_PIO0);
    gpio_set_dir(DROPPER2_GATE, OUT);
    return;
}

void init_gpio(){
    //LED GPIO setup
    gpio_set_function(LED_1, GPIO_FUNC_PIO0); 
    gpio_set_function(LED_2, GPIO_FUNC_PIO0); 
    gpio_set_dir(LED_1,OUT);
    gpio_set_dir(LED_2,OUT);
    //call to set gpio
    gpio_put(LED_1,false);
    gpio_put(LED_2,false);
    return;
}

void init_i2c(){
    // I2C Initialisation. Using it at 400Khz.
    i2c_init(I2C_PORT, 400*1000);
    
    //I2C GPIO setup using pins from board cna be changed in the definitions at the top for the ports
    gpio_set_function(I2C_SDA, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA);
    gpio_pull_up(I2C_SCL);
    return;

}

void init_irq(){
    //Interrupt 1 can use another pin for the interrupt
    //irq_set_enabled(I2C0_IRQ, ENABLE);
    //irq_set_priority(I2C0_IRQ, HIGHPRIOR);
    //Interrupt 2
   // irq_set_enabled(I2C1_IRQ, ENABLE);
    //irq_set_priority(I2C0_IRQ, HIGHPRIOR+1);

    //Test to check if interruput is active
    //This example calls a hard stop when the interrupt is triggered
    //if(irq_is_enabled(I2C0_IRQ)){
        //puts("Interrupt is enabled turn on hard stop");
       // hard_stop();

   // }
    return;
}