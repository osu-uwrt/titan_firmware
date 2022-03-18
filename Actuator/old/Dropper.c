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

void drop_marker(uint dropper){
    //Smple IO activate then delay and turn off and exit
    //High for 500ms then turn off need to take in an argurment for which dropper
    if(dropper==1){
        printf("Dropping marker 1 \n");
        gpio_put(DROPPER1_GATE,true);
        sleep_ms(500);
        printf("Marker 1 dropped \n");
        gpio_put(DROPPER1_GATE,false);
    }
    else if(dropper==2){
        printf("Dropping marker 2 \n");
        gpio_put(DROPPER2_GATE,true);
        sleep_ms(500);
        printf("Marker 2 dropped \n");
        gpio_put(DROPPER2_GATE,false);
    }
    else if(dropper==0){
        printf("Dropper is off \n");
        gpio_put(DROPPER1_GATE,false);
        gpio_put(DROPPER2_GATE,false);
    }
    else{
        printf("Invalid input dropper is off \n");
        gpio_put(DROPPER1_GATE,false);
        gpio_put(DROPPER2_GATE,false);
    }
    return;
}