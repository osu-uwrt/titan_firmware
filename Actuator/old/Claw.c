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


void old_claw_setup(uint16 duty, uint32 f_pwm){
/*	//const uint32 f_pwm = 400; // frequency we want to generate
    //60% duty cycle for a 1500usec high
	//const uint duty = 60; // duty cycle, in percent

	uint slice_num = pwm_gpio_to_slice_num(PWM_CLAW); // get PWM slice for GPIO 0 (it's slice 0)


	// set frequency
	// determine top given Hz - assumes free-running counter rather than phase-correct
    uint32 f_sys = clock_get_hz(clk_sys); // typically 125'000'000 Hz
    float divider = (float)(f_sys / 1000000UL);  // let's arbitrarily choose to run pwm clock at 1MHz

    //Start here we need to adjust the 1MHz value to something else becuase the following function hangs becuase of an invalid divider line
    
    //debug code
    printf("fsys %d\n", f_sys);
    printf("divider %f\n", divider);
    printf("slice num %d\n", slice_num);

    //pwm_set_clkdiv(slice_num, divider); // pwm clock should now be running at 1MHz Just this line breaks it
    uint16 top = (short) ((1000000UL/f_pwm) -1UL); // TOP is u16 has a max of 65535, being 65536 cycles
    //printf("top %s\n", top);
    pwm_set_wrap(slice_num, top);

	 // set duty cycle
    uint16 level = (short)((top+1) * duty / 100 -1); // calculate channel level from given duty cycle in %
    //printf("level %s\n", level);
    pwm_set_chan_level(slice_num, 0, level); 
	
    pwm_set_enabled(slice_num, true); // let's go!
    return; */

    uint slice_num = pwm_gpio_to_slice_num(PWM_CLAW);
    uint32 f_sys = clock_get_hz(clk_sys); // typically 125'000'000 Hz
    float divider = (float)(f_sys / 1000000UL);  // let's arbitrarily choose to run pwm clock at 1MHz
    uint16 top = (short) ((1000000UL/f_pwm) -1UL); // TOP is u16 has a max of 65535, being 65536 cycles
    //init_pwm(top);
     pwm_set_clkdiv(slice_num, divider);
    pwm_set_wrap(slice_num, top);
    
	 // set duty cycle
    uint16 level = (short)((top+1) * duty / 100 -1); // calculate channel level from given duty cycle in %
    pwm_set_chan_level(slice_num, 0, level); 
	
    pwm_set_enabled(slice_num, true); // let's go!
    return;
}

void set_old_claw(int state, uint hardStoper){
    //This claw is from blue robotics
    //https://bluerobotics.com/store/rov/bluerov2-accessories/newton-gripper-asm-r2-rp/
    //Here we will just need to send the pwm pulses listed in the website to the pin
    /* 2: Actively closing
     *  0: Stopped
     *  1: Actively Opening
     */
        uint32 f_pwm = 400;
        //need 1500usec for pause
        const uint16 off_duty = 60;
        //need 1200usec for close
        const uint16 close_duty = 48;
        //need 1800usec for close
        const uint16 open_duty = 72;

    if(state==0||hardStoper==1){

        old_claw_setup(off_duty,f_pwm);
    }
    else if(state==1){

        old_claw_setup(open_duty,f_pwm);
    }
    else if(state==2){

        old_claw_setup(close_duty,f_pwm);
    }
    else{
        //Stops claw since state got garbage
        printf("Error occured when setting old claw position");
        printf("State was invalid and state was %d\n", state);
        return;
    }
    printf("Setting old claw state to %d\n", state);
    return;
}

//Pass a parameter n that can stop it
void set_new_claw(int state, uint hardStoper){
    //We will have to send a raw pwm singal since this is a custom device
    /* 2: Actively closing
     *  0: Stopped
     *  1: Actively Opening
     */
    //high on one pin low and other for opening
    //low on one pin high on other for closing
    //neutral is low on both
    if (state==1){
        gpio_put(PWM_DRIVER1,true);
        gpio_put(PWM_DRIVER2,false);
    }
    else if(state==2){
        gpio_put(PWM_DRIVER1,false);
        gpio_put(PWM_DRIVER2,true);
    }
    else if(state==0||hardStoper==1){
        gpio_put(PWM_DRIVER1,false);
        gpio_put(PWM_DRIVER2,false);
    }
    else{
        //Stops claw since state got garbage
        printf("Error occured when setting new claw position");
        printf("State was %d\n", state);
        return;
    }

    printf("Setting new claw state to %d\n", state);
    return;
}

//Fucntion called for the hard stop interrupt
void hard_stop(){
    //Fucntion should just be called in the intruppt that calls this and 
    set_new_claw(0,1);
    set_old_claw(0,1);
    //Rest state vairbale that was being used in fucntion to 0 could be the local or global variable, whichever is passed to the function
    puts("HardStop was triggered");
    return;
}