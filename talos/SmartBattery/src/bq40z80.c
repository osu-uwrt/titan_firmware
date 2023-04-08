#include "bq40z80.h"
#include "hardware/gpio.h"
#include "pio_i2c.h"
#include <stdio.h>

#define RETRANSMIT_COUNT 4

uint8_t BQ_LEDS[3] = {LED_R_PIN, LED_Y_PIN, LED_G_PIN};
uint pio_12c_program;

void  __attribute__ ((noinline)) bq_handle_i2c_transfer(uint8_t* bq_reg, uint8_t* rx_buf, uint len){
    int ret_code = 0;
    uint retries = 0;

    while(ret_code && retries < RETRANSMIT_COUNT){
        // this is a bug in the SM
        i2c_program_init(pio0, PIO_SM, pio_12c_program, BMS_SDA_PIN, BMS_SCL_PIN);

        // send the request to the chip
        ret_code |= pio_i2c_write_blocking(pio0, PIO_SM, BQ_ADDR, bq_reg, 1);
        ret_code |= pio_i2c_read_blocking(pio0, PIO_SM, BQ_ADDR, rx_buf, len);
    }

    if(ret_code != 0){
        //something bad happened to the i2c, panic
        printf("%d", ret_code);
        panic("PIO I2C NACK during transfer %d times", RETRANSMIT_COUNT);
    }
}

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
    // bq_update_soc_leds();
    // finish chip init
    return 0;
}

uint8_t bq_pack_present(){
    // read the operationstatus register (32 bits)
    uint8_t data[4] = {0, 0, 0, 0};
    uint8_t reg_addr[1] = {BQ_READ_OPER_STAT};
    bq_handle_i2c_transfer(reg_addr, data, 4);

    //remake the 32 bit value
    uint32_t oper_data = 0;
    for(uint8_t i = 0; i < 4; i++){
        oper_data |= (data[i] << (8 * i));
    }

    // test the presence bit (bit 1)
    return (uint8_t)(oper_data & 0x01);
}

uint8_t bq_pack_soc() {
    // read the SOC from the pack
    uint8_t data[1] = {0};
    uint8_t reg_addr[1] = {BQ_READ_RELAT_SOC};
    bq_handle_i2c_transfer(reg_addr, data, 1);

    return data[0];
}

void bq_update_soc_leds(){
    uint8_t data = bq_pack_soc();

    uint8_t state = 0;
    if(data > WARN_SOC_THRESH ){
        state = 2;
    } else if (data > STOP_SOC_THRESH){
        state = 1;
    }

    for(uint8_t led = 0; led < 3; led++) {
        gpio_put(BQ_LEDS[led], led == state);
    }

}

uint16_t bq_pack_voltage(){
    // read the SOC from the pack
    uint8_t data[2] = {0, 0};
    uint8_t reg_addr[1] = {BQ_READ_PACK_VOLT};
    bq_handle_i2c_transfer(reg_addr, data, 2);
    

    uint16_t millivolts = data[0] | data[1] << 8;
    return millivolts;
}

int16_t bq_pack_current(){
    // read the SOC from the pack
    uint8_t data[2] = {0, 0};
    uint8_t reg_addr[1] = {BQ_READ_PACK_CURR};
    bq_handle_i2c_transfer(reg_addr, data, 2);

    int16_t current_ma = data[0] | data[1] << 8;
    return current_ma;
}

int16_t bq_avg_current(){
    // read the SOC from the pack
    uint8_t data[2] = {0, 0};
    uint8_t reg_addr[1] = {BQ_READ_AVRG_CURR};
    bq_handle_i2c_transfer(reg_addr, data, 2);

    int16_t current_ma = data[0] | data[1] << 8;
    return current_ma;
}

uint16_t bq_time_to_empty(){
    // read the SOC from the pack
    uint8_t data[2] = {0, 0};
    uint8_t reg_addr[1] = {BQ_READ_TIME_EMPT};
    bq_handle_i2c_transfer(reg_addr, data, 2);

    uint16_t runtime_mins = data[0] | data[1] << 8;
    return runtime_mins;
}

struct bq_pack_info_t bq_pack_mfg_info(){
    struct bq_pack_info_t pack_info;

    // read the mfg serial #
    uint8_t data[21] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
    uint8_t reg_addr[1] = {BQ_READ_CELL_DATE};
    bq_handle_i2c_transfer(reg_addr, data, 2);

    // capture the manufacture date and make it more usable
    uint16_t cell_date_raw = data[0] | data[1] << 8;
    pack_info.mfg_day = cell_date_raw & 0x1F;
    pack_info.mfg_mo = (cell_date_raw >> 5) & 0x0F;
    pack_info.mfg_year = (cell_date_raw >> 9) + 1980;

    // read the SOC from the pack
    reg_addr[0] = BQ_READ_CELL_SERI;
    bq_handle_i2c_transfer(reg_addr, data, 2);

    pack_info.serial = data[0] | data[1] << 8;

    // read the SOC from the pack
    reg_addr[0] = BQ_READ_CELL_NAME;
    bq_handle_i2c_transfer(reg_addr, data, 21);

    // move the info from the pack read back into the mfg data
    for(uint8_t i = 1; i < 21; i++){
        pack_info.name[i-1] = data[i];
    }
    
    // now kick it all out
    return pack_info;
}