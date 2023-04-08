#include <stdio.h>

#include "pico/stdlib.h"

#include "basic_logger/logging.h"
#include "build_version.h"

#include "bq40z80.h"

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

    struct bq_pack_info_t bq_info = bq_pack_mfg_info();


    //try to contact the chip
    while(true){
        // update pack SOC
        bq_update_soc_leds();

        // read pack status
        uint16_t voltage = bq_pack_voltage();
        uint8_t present = bq_pack_present();

        printf("Present: %02x, voltage: %d\n", present, voltage);

        // detect presence for shutdown
        if(present == 0){
            not_present_count ++;
            if(not_present_count > NOT_PRESENT_THRESH){
                printf("No presence detected. Shutting down!\n");
                gpio_put(PWR_CTRL_PIN, 0);
            }
        }
        sleep_ms(1000);
    }

    return 0;
}