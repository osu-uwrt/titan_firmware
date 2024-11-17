#include "ledc_commands.h"

#include "hardware/adc.h"
#include "hardware/clocks.h"
#include "hardware/gpio.h"
#include "hardware/pwm.h"
#include "hardware/spi.h"
#include "hardware/timer.h"
#include "pico/binary_info.h"
#include "pico/stdlib.h"
#include "pico/time.h"
#include "titan/debug.h"

#include <inttypes.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>

#define LEDC_SPI_INST __CONCAT(spi, LEDC_SPI)

// LEDC controller defines in header

#define OPCODE_WRITE 0b00
#define OPCODE_READ 0b01
#define OPCODE_READ_CLR 0b10
#define OPCODE_READ_ROM 0b11

#define THERMISTOR_SERIES_RESISTANCE 10000.0
#define THERMISTOR_V_REF 3.3
#define THERMISTOR_R_25 22000.0
#define THERMISTOR_B_25_85 3730.0
#define THERMISTOR_NOMINAL_TEMP 298.15

const uint led_r_path[2] = { LEDC1, BUCK1 };
const uint led_g_path[2] = { LEDC1, BUCK2 };
const uint led_b_path[2] = { LEDC2, BUCK2 };
const uint led_w_path[2] = { LEDC2, BUCK1 };

bool do_periodic_spi = true;
uint32_t write_fail = 0;

static uint32_t spi_xfer(uint target, uint32_t data, uint8_t *gs_out) {
    uint8_t tx_packet[] = { (data >> 24) & 0xFF, (data >> 16) & 0xFF, (data >> 8) & 0xFF, data & 0xFF };
    uint8_t rx_packet[sizeof(tx_packet)];
    uint cs_pin = target == LEDC1 ? LEDC_NCS1_PIN : LEDC_NCS2_PIN;

    gpio_put(cs_pin, 0);
    busy_wait_us(1);
    if (spi_write_read_blocking(LEDC_SPI_INST, tx_packet, rx_packet, sizeof(tx_packet)) != 4) {
        // TODO: Something a little less agressive than this
        panic("LEDC SPI Xfer Failure!");
    }
    busy_wait_us(1);
    gpio_put(cs_pin, 1);

    if (gs_out) {
        *gs_out = rx_packet[0];
    }

    return ((uint32_t) (rx_packet[1]) << 16) | ((uint32_t) (rx_packet[2]) << 8) | rx_packet[3];
}

// Wrappers around SPI logic
#define spi_xfer_cmd(target, opcode, addr, data, gs_out)                                                               \
    spi_xfer((target), ((opcode) << 30) | (((addr) & 0x3F) << 24) | ((data) & 0xFFFFFF), (gs_out))
#define spi_read(target, addr, gs_out) spi_xfer_cmd(target, OPCODE_READ, addr, 0, gs_out)
#define spi_read_clr(target, addr, gs_out) spi_xfer_cmd(target, OPCODE_READ_CLR, addr, 0, gs_out)
#define spi_read_rom(target, addr, gs_out) spi_xfer_cmd(target, OPCODE_READ_ROM, addr, 0, gs_out)
#define spi_write(target, addr, data, gs_out) spi_xfer_cmd(target, OPCODE_WRITE, addr, data, gs_out)

static uint32_t correct_parity_bit(uint32_t val, bool odd_parity) {
    // ensure the parity bit at the end of the 24 bit spi is correctly computed
    // ODD parity bit check

    uint32_t val_count = val;

    // count the number of ones in the 24 bit int
    int ones_count = 0;
    for (int n = 23; n >= 1; n = n - 1) {
        if (val_count - pow(2, n) >= 0) {
            ones_count++;
            val_count = val_count - pow(2, n);
        }
    }

    if (odd_parity) {
        if (!(ones_count % 2)) {
            return (val | (1 << 0));
        }

        return (val & 4294967294);
    }

    if ((ones_count % 2)) {
        return (val | (1 << 0));
    }
    return (val & 4294967294);
}

bool controller_satisfy_watchdog(__unused repeating_timer_t *t) {
    if (!do_periodic_spi)
        return true;

    static bool watchdog_state = false;

    uint8_t gs;

    uint32_t val = 45;
    static uint32_t previous_val = 0;

    if (watchdog_state) {
        spi_write(LEDC1, 0x04, 0, &gs);
        spi_write(LEDC2, 0x04, 0, &gs);

        val = spi_read(LEDC1, 0x04, &gs);
    }
    else {
        spi_write(LEDC1, 0x04, 8388609, &gs);
        spi_write(LEDC2, 0x04, 8388609, &gs);

        val = spi_read(LEDC1, 0x04, &gs);
    }

    if (val == previous_val)
        write_fail++;

    previous_val = val;
    watchdog_state = !watchdog_state;

    return true;
}

void controller_clear_watchdog_error(uint controller) {
    uint8_t gs;
    spi_read_clr(controller, 0x06, &gs);
}

void controller_enable(uint controller) {
    uint8_t gs;

    do_periodic_spi = false;

    // Unlock
    sleep_ms(1);
    uint32_t val = spi_read(controller, 0x01, &gs);
    val = val | (1 << 3);
    val = val | (1 << 2);
    val = correct_parity_bit(val | (1 << 1), false);
    sleep_ms(1);
    spi_write(controller, 0x01, val, &gs);
    sleep_ms(1);

    // Set enabled high and gostby low
    sleep_ms(1);
    val = spi_read(controller, 0x02, &gs);
    val = correct_parity_bit(val | (1 << 2), false);
    sleep_ms(1);
    spi_write(controller, 0x02, val, &gs);
    sleep_ms(1);

    do_periodic_spi = true;
    // watchdog_state = !watchdog_state
}

static void buck_set_brightness(uint controller, uint buck, uint brightness) {
    uint8_t gs;

    uint32_t spi_val = spi_read(controller, 0x01, &gs);

    spi_val &= buck == BUCK1 ? 0x3FFF : 0xFFC00F;
    spi_val |= brightness << (buck == BUCK1 ? 14 : 4);

    spi_write(controller, 0x01, correct_parity_bit(spi_val, false), &gs);
}

void buck_set_control_mode(uint controller, uint buck, uint mode) {
    uint8_t gs;

    uint32_t spi_val = spi_read(controller, 0x03, &gs);

    spi_val &= buck == BUCK1 ? 0xFF3FFF : 0xFFCFFF;
    spi_val |= mode << (buck == BUCK1 ? 14 : 12);

    spi_write(controller, 0x03, correct_parity_bit(spi_val, true), &gs);
}

void buck_set_peak_current(uint controller, uint buck, uint current) {
    uint8_t gs;

    uint32_t spi_val = spi_read(controller, 0x02, &gs);

    spi_val &= buck == 1 ? 0x3FFFF : 0xFC0FFF;
    spi_val |= current << (buck == 1 ? 18 : 12);

    spi_write(controller, 0x02, correct_parity_bit(spi_val, false), &gs);
}

static void rgb_to_rgbw(uint *r, uint *g, uint *b, uint *w) {
    *w = MIN(*r, MIN(*g, *b));

    *r -= *w;
    *g -= *w;
    *b -= *w;
}

void led_set_rgb(uint r, uint g, uint b, float maxBrightness) {
    uint w;
    rgb_to_rgbw(&r, &g, &b, &w);

    buck_set_brightness(led_r_path[0], led_r_path[1], r * (1023.0 / 255.0) * maxBrightness);
    buck_set_brightness(led_g_path[0], led_g_path[1], g * (1023.0 / 255.0) * maxBrightness);
    buck_set_brightness(led_b_path[0], led_b_path[1], b * (1023.0 / 255.0) * maxBrightness);
    buck_set_brightness(led_w_path[0], led_w_path[1], w * (1023.0 / 255.0) * maxBrightness);
}

float al_read_temp() {
    // read led al board thermistor

    adc_select_input(0);
    const float convert_to_voltage_factor = 3.3f / (1 << 12);
    float voltage = adc_read() * convert_to_voltage_factor;

    // calculate thermistor resistance
    float resistance = THERMISTOR_SERIES_RESISTANCE * (voltage) / (THERMISTOR_V_REF - voltage);

    float temperature =
        1 / (log(resistance / THERMISTOR_R_25) / THERMISTOR_B_25_85 + 1 / THERMISTOR_NOMINAL_TEMP) - 273.0;

    return temperature;
}

void init_spi_and_gpio() {
    // SPI Init
    bi_decl_if_func_used(bi_3pins_with_func(LEDC_MISO_PIN, LEDC_MOSI_PIN, LEDC_SCK_PIN, LEDC_SPI));
    spi_init(LEDC_SPI_INST, 4000000);  // Run SPI at 4 MHz
    spi_set_format(LEDC_SPI_INST,      // SPI instance
                   8,                  // Number of bits per transfer
                   0,                  // Polarity (CPOL)
                   0,                  // Phase (CPHA)
                   SPI_MSB_FIRST);     // MSB First
    gpio_set_function(LEDC_MISO_PIN, GPIO_FUNC_SPI);
    gpio_set_function(LEDC_SCK_PIN, GPIO_FUNC_SPI);
    gpio_set_function(LEDC_MOSI_PIN, GPIO_FUNC_SPI);

    // Configure the CS pins
    bi_decl_if_func_used(bi_1pin_with_name(LEDC_NCS1_PIN, "LEDC 1 nCS Pin"));
    bi_decl_if_func_used(bi_1pin_with_name(LEDC_NCS2_PIN, "LEDC 2 nCS Pin"));
    gpio_init(LEDC_NCS1_PIN);
    gpio_put(LEDC_NCS1_PIN, 1);
    gpio_set_dir(LEDC_NCS1_PIN, GPIO_OUT);
    gpio_init(LEDC_NCS2_PIN);
    gpio_put(LEDC_NCS2_PIN, 1);
    gpio_set_dir(LEDC_NCS2_PIN, GPIO_OUT);

    gpio_init(LEDC_DIN1_PIN);
    gpio_set_dir(LEDC_DIN1_PIN, GPIO_OUT);

    gpio_init(LEDC_DIN2_PIN);
    gpio_set_dir(LEDC_DIN2_PIN, GPIO_OUT);

    // provide pwm clock
    gpio_set_function(LEDC_PWM_CLK, GPIO_FUNC_PWM);

    // Find out which PWM slice is connected to GPIO 0 (it's slice 0)
    uint slice_num = pwm_gpio_to_slice_num(LEDC_PWM_CLK);
    uint chan = pwm_gpio_to_channel(LEDC_PWM_CLK);

    // Want PWMCLK to be a square wave at 204,800 Hz
    // Set period of 4 cycles (0 to 3 inclusive)
    pwm_set_wrap(slice_num, 3);
    // Set channel A output high for two cycles before dropping (50% duty cycle)
    pwm_set_chan_level(slice_num, chan, 2);
    // This means that we need the slice to be clocked at 819,200 Hz
    // Compute fractioanl divider to get our target frequency
    pwm_set_clkdiv(slice_num, clock_get_hz(clk_sys) / 819200.0f);
    // Set the PWM running
    pwm_set_enabled(slice_num, true);

    // init adc for read thermistor
    adc_init();
    adc_gpio_init(TEMP_SENSE_PIN);
}

// Split canmore commands into a separate file for cleanliness
#include "ledc_canmore_commands.h"
