#include <stdio.h>

#include "pico/stdlib.h"

#include "basic_logger/logging.h"
#include "build_version.h"

#include "BQ40Z80.h"

// time in seconds for pack to not be present before sleeping
#define NOT_PRESENT_THRESH 10

int not_present_count = 0;

int main(){
    stdio_init_all();
    
    //init GPIO
    // Latch RP2040 power to on
    gpio_init(PWR_CTRL_PIN);
    gpio_set_dir(PWR_CTRL_PIN, GPIO_OUT);
    gpio_put(PWR_CTRL_PIN, 1);

    // Initialize stdio
    stdio_init_all();
    LOG_INFO("%s", FULL_BUILD_TAG);

    //try init bq40z80
    if(bq_init() > 0){
        panic("Failed to init BMS Chip");
    }
    printf("BQ init success!\n");


    //try to contact the chip
    while(true){
        // update pack SOC
        bq_update_soc_leds();

        // try to init the chip and read a serial#
        uint8_t present = bq_pack_present();
        if(present == 0){
            printf("Pack not present!\n");

            not_present_count ++;

            if(not_present_count > NOT_PRESENT_THRESH){
                printf("No presence detected. Shutting down!\n");
                gpio_put(PWR_CTRL_PIN, 0);
            }
        } else {
            printf("Pack present!\n");
        }
        sleep_ms(1000);
    }

    return 0;
}