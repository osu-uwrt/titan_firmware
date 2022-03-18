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


//NOTES
//Start looking in i2c
//i2c example
//interput usage on pico sdk and add in an example code that takes an i2c communction that triggers an inttrupet that changes a gobal varaible 
//THe interrupt global should be then refernced in a local variable in main and passed into the fucntions to do stuff
//https://www.digikey.com/en/maker/projects/raspberry-pi-pico-rp2040-i2c-example-with-micropython-and-cc/47d0c922b79342779cdbd4b37b7eb7e2


// int64_t alarm_callback(alarm_id_t id, void *user_data) {
//     //Put your timeout handler code in here
//    return 0;
// }


//Fucntion called for the hard stop interrupt


void test_sel();

int main()
{
    //Initiaization functions for gpio and such
    stdio_init_all();
        sleep_ms(5000);
        puts("1");
    //init_i2c();

        puts("2");
    //init_pwm(); //NEW CLAW isn't pwm. Also this function gets called within the set old claw function now sincewe needed to pass a varible in to configure it
    uint32 f_pwm = 400;
    uint16 top = (short) ((1000000UL/f_pwm) -1UL); // TOP is u16 has a max of 65535, being 65536 cycles
    init_pwm(top);
        puts("3");
    //init_gpio();

       puts("4");
    //init_irq();

        puts("5");
    init_dropper();

        puts("6");
    init_new_claw();
        sleep_ms(5000);
        puts("7");

    // Timer example code - This example fires off the callback after 2000ms
    //add_alarm_in_ms(2000, alarm_callback, NULL, false);

    while (1){
        test_sel();
    };

    return 0;
}

void test_sel(){
    char str1[20], str2[20];
    #define LENGTH 20
    int i;
    puts("Do you want claw or dropper use space to end");
    i=0;
        
    while(i<20){
        
    str1[i]='\0';
    str2[i]='\0';
    
    ++i;
    }
    i=0;
    while(i<20){
        
        scanf("%c", &str1[i]);
        printf("%c", str1[i]);
        if(str1[i]==' '){
            i=20;
            puts("\n");
        }
    
        ++i;
    }

    if(strcmp(str1, "claw ") == 0){
        puts("You Selected claw");
        sleep_ms(100);
        puts("Use 0 to stop    1 to open     2 to close      end with a space");
        i=0;
        while(i<20){
            scanf("%c", &str2[i]);
            printf("%c", str2[i]);
            if(str2[i]==' '){
               i=20;
                puts("\n");
            }
            ++i;
        }
        if(str2[0]=='2'){
            puts("running in reverse");
            set_old_claw(2,0);
        }
        else if(str2[0]=='1'){
            puts("running forward");
            set_old_claw(1,0);
        }
        else if(str2[0]=='0'){
           puts("Stopping");
            set_old_claw(0,0);
        }
        else{
            puts("Input invalid");
            sleep_ms(100);  
        }
    }
    else if(strcmp(str1, "dropper ") == 0){
        puts("You Selected dropper");
        sleep_ms(100);
        puts("Use 0 to do nothing    1 for dropper 1     2 for dropper 2      end with a space");
        i=0;
        while(i<20){
            scanf("%c", &str2[i]);
            printf("%c", str2[i]);
            if(str2[i]==' '){
               i=20;
                puts("\n");
            }
            ++i;
        }
        if(str2[0]=='2'){
            drop_marker(2);
        }
        else if(str2[0]=='1'){
            drop_marker(1);
        }
        else if(str2[0]=='0'){
            drop_marker(0);
        }
        else{
            puts("Input invalid");
            sleep_ms(100);  
        }
    }
    else{
        puts("Input invalid");
        //printf("strcomp is %d\n", strcmp(str1, "claw "));
        sleep_ms(100);
    }
    //printf("str 1 is %s\n",str1);
    //printf("str 2 is %s\n",str2);


    //printf("str 1 clear is %s\n",str1);
    //printf("str 2 clear is %s\n",str2);

}