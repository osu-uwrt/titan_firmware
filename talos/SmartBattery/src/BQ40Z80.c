#include "BQ40Z80.h"
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "pio_i2c.h"

uint8_t BQ_LEDS[3] = {LED_R_PIN, LED_Y_PIN, LED_G_PIN};
uint pio_12c_program;

uint8_t bq_init() {
    uint8_t retries = 0;
        
    // Init the wake pin, active high
    gpio_init(BMS_WAKE_PIN);
    gpio_set_dir(BMS_WAKE_PIN, GPIO_OUT);
    gpio_put(BMS_WAKE_PIN, 0);

    for(uint8_t led = 0; led < 3; led++) {
        gpio_init(BQ_LEDS[led]);
        gpio_set_dir(BQ_LEDS[led], GPIO_OUT);
        gpio_put(BQ_LEDS[led], 0);
    }

    // init PIO I2C
    pio_12c_program = pio_add_program(pio0, &i2c_program);
    i2c_program_init(pio0, PIO_SM, pio_12c_program, BMS_SDA_PIN, BMS_SCL_PIN);

    // make the request for the serial #
    uint8_t data[4] = {BQ_READ_CELL_SERI, 0x00};

    // Start the I2C to the chip
    while((data[1] == 0x00 || data[1] == 0xFF) && retries < 3) {
        int ret_code = 0;
        
        // send the request to the chip
        ret_code |= pio_i2c_write_blocking(pio0, PIO_SM, BQ_ADDR, data, 1);
        ret_code |= pio_i2c_read_blocking(pio0, PIO_SM, BQ_ADDR, data, 2);

        // Check for valid data
        if(ret_code != 0 || data[1] == 0x00 || data[1] == 0xFF) {
            gpio_put(BMS_WAKE_PIN, 1);
            sleep_ms(1000);
            gpio_put(BMS_WAKE_PIN, 0);
            retries++;
        } else {
            // if we get here, we found a valid bq40z80
            break;
        }

    }
    if(retries == 3)
        return retries;


    bq_update_soc_leds();
    // finish chip init
    return 0;
}

uint8_t bq_pack_present(){
    int ret_code = 0;

    // shouldnt have to do this, but there is a glitch in the SM @rjp5th you might want to have a look with this
    i2c_program_init(pio0, PIO_SM, pio_12c_program, BMS_SDA_PIN, BMS_SCL_PIN);

    // read the operationstatus register (32 bits)
    uint8_t data[4] = {BQ_READ_OPER_STAT, 0, 0, 0};
    ret_code |= pio_i2c_write_blocking(pio0, PIO_SM, BQ_ADDR, data, 1);
    ret_code |= pio_i2c_read_blocking(pio0, PIO_SM, BQ_ADDR, data, 4);
    if(ret_code != 0){
        //something bad happened to the i2c, panic
        panic("PIO I2C failure during pack present test");
    }

    //remake the 32 bit value
    uint32_t oper_data = 0;
    for(uint8_t i = 0; i < 4; i++){
        oper_data |= (data[i] << (8 * i));
    }

    // test the presence bit (bit 1)
    return (uint8_t)(oper_data & 0x01);
}

void bq_update_soc_leds(){
    int ret_code = 0;

    // shouldnt have to do this, but there is a glitch in the SM @rjp5th you might want to have a look with this
    i2c_program_init(pio0, PIO_SM, pio_12c_program, BMS_SDA_PIN, BMS_SCL_PIN);

    // read the SOC from the pack
    uint8_t data[1] = {BQ_READ_RELAT_SOC};
    ret_code |= pio_i2c_write_blocking(pio0, PIO_SM, BQ_ADDR, data, 1);
    ret_code |= pio_i2c_read_blocking(pio0, PIO_SM, BQ_ADDR, data, 1);

    uint8_t state = 0;
    if(data[0] > 60 ){
        state = 2;
    } else if (data[0] > 30){
        state = 1;
    }

    for(uint8_t led = 0; led < 3; led++) {
        gpio_put(BQ_LEDS[led], led == state);
    }

}