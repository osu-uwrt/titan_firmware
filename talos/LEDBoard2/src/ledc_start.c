#include "ledc_start.h"

#include "hardware/adc.h"
#include "hardware/clocks.h"
#include "hardware/gpio.h"
#include "hardware/pwm.h"
#include "hardware/spi.h"
#include "pico/binary_info.h"
#include "pico/stdlib.h"

// set up pins and pwm
// put chips into active mode
// other hardware start up

void init_spi(void) {
    gpio_set_function(LEDC_MISO_PIN, GPIO_FUNC_SPI);
    gpio_set_function(LEDC_SCK_PIN, GPIO_FUNC_SPI);
    gpio_set_function(LEDC_MOSI_PIN, GPIO_FUNC_SPI);
    spi_init(LEDC_SPI_INST, 4000000);
    spi_set_format(LEDC_SPI_INST,   // spi instance
                   8,               // bits per transfer
                   0,               // polarity
                   0,               // phase
                   SPI_MSB_FIRST);  // big endian
}

void init_pwm(void) {
    gpio_set_function(LEDC_PWM_CLK, GPIO_FUNC_PWM);
    uint slice_num = pwm_gpio_to_slice_num(LEDC_PWM_CLK);
    uint chan = pwm_gpio_to_channel(LEDC_PWM_CLK);
    // just need a 50% duty cycle so we'll make it easy
    pwm_set_wrap(slice_num, 3);
    pwm_set_chan_level(slice_num, chan, 2);
    pwm_set_clkdiv(slice_num, clock_get_hz(clk_sys) / 819200.0f);
    pwm_set_enabled(slice_num, true);
}

void init_hardware(void) {
    bi_decl_if_func_used(bi_3pins_with_func(LEDC_MISO_PIN, LEDC_MOSI_PIN, LEDC_SCK_PIN, LEDC_SPI));
    gpio_init(LEDC_DIN1_PIN);
    gpio_init(LEDC_DIN2_PIN);

    bi_decl_if_func_used(bi_1pin_with_name(LEDC_NCS1_PIN, "LEDC 1 nCS Pin"));
    bi_decl_if_func_used(bi_1pin_with_name(LEDC_NCS2_PIN, "LEDC 2 nCS Pin"));
    gpio_init(LEDC_NCS1_PIN);
    gpio_put(LEDC_NCS1_PIN, 1);
    gpio_set_dir(LEDC_NCS1_PIN, GPIO_OUT);
    gpio_init(LEDC_NCS2_PIN);
    gpio_put(LEDC_NCS2_PIN, 1);
    gpio_set_dir(LEDC_NCS2_PIN, GPIO_OUT);

    adc_init();
    adc_gpio_init(TEMP_SENSE_PIN);

    init_spi();
    init_pwm();

    // need to do chip wakeup first implement spi comms

    // sect 4.2 SPI communication, then Command Byte
    // Dimming with integrated PWM generator
    // sect 3.1 Operating modes
    // Buck converter’s start-up
    // look at spi timings in datasheet
    // Steinhart–Hart equation for reading temp, first convert to voltage from 12bit
}

// spi data frames are 4 bytes
