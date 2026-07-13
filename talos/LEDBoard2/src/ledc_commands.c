#include "ledc_commands.h"

#include "ledc_io.h"
#include "ledc_registers.h"
#include "shifts_and_masks.h"

#include "hardware/adc.h"
#include "pico/time.h"

#include <math.h>
#include <stdint.h>
#include <string.h>

bool is_active_mode = false;

void pet_watchdog() {
    if (is_active_mode) {
        for (uint8_t controller = LEDC1; controller <= LEDC2; controller++) {
            cr4_t cr4 = decode_register(spi_read(controller, CR4_ADDRESS));
            cr4 ^= (0b1 << WATCHDOG_BIT_SHIFT);
            spi_write(controller, CR4_ADDRESS, cr4);
        }
    }
}

void enable_dimming() {
    cr3_fields_t cr3 = { 0 };
    cr3.din_map1 = 1;
    cr3.din_map2 = 1;
    spi_write(LEDC1, CR3_ADDRESS, encode_cr3(&cr3));
    spi_write(LEDC2, CR3_ADDRESS, encode_cr3(&cr3));
}

void set_controllers_active_mode() {
    cr1_fields_t cr1 = decode_cr1(decode_register(spi_read(LEDC1, CR1_ADDRESS)));  // was nulled
    // cr1_fields_t cr1 = { 0 };

    // first frame, unlock
    cr1.unlock = 1;
    spi_write(LEDC1, CR1_ADDRESS, encode_cr1(&cr1));
    cr1 = decode_cr1(decode_register(spi_read(LEDC2, CR1_ADDRESS)));  // new line
    spi_write(LEDC2, CR1_ADDRESS, encode_cr1(&cr1));

    sleep_ms(1);
    // second frame, set enable high and gostby low
    // may need to read the register and preserve reserved bit
    cr2_fields_t cr2 = decode_cr2(decode_register(spi_read(LEDC1, CR2_ADDRESS)));
    cr2.enable = 1;
    cr2.gostby = 0;
    spi_write(LEDC1, CR2_ADDRESS, encode_cr2(&cr2));

    cr2 = decode_cr2(decode_register(spi_read(LEDC2, CR2_ADDRESS)));
    cr2.enable = 1;
    cr2.gostby = 0;
    spi_write(LEDC2, CR2_ADDRESS, encode_cr2(&cr2));
    is_active_mode = true;
    // for (uint8_t controller = LEDC1; controller <= LEDC2; i++) {
    //     cr1_fields_t cr1 = decode_cr1(decode_register(spi_read(controller, CR1_ADDRESS)));

    // }
}

void set_peak_current() {
    for (uint8_t controller = LEDC1; controller <= LEDC2; controller++) {
        cr2_fields_t cr2 = decode_cr2(decode_register(spi_read(controller, CR2_ADDRESS)));
        cr2.il1_peak = cr2.il2_peak = BUCK_PEAK_CURRENT;
        spi_write(controller, CR2_ADDRESS, encode_cr2(&cr2));
        sleep_ms(1);
    }
}

float read_temp() {
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

// void enable_controller(uint8_t ledc) {
//     cr1_fields_t cr1 = { 0 };
//     cr2_fields_t cr2 = { 0 };

//     // first frame, unlock
//     cr1.unlock = 1;
//     cr1_t enc_cr1 = encode_cr1(&cr1);
//     sleep_ms(1);
//     spi_write(ledc, CR1_ADDRESS, enc_cr1);
//     sleep_ms(1);

//     // second frame, set enable high and gostby low
//     cr2.enable = 1;
//     cr2.gostby = 0;
//     cr2_t enc_cr2 = encode_cr2(&cr2);
//     sleep_ms(1);
//     spi_write(ledc, CR2_ADDRESS, enc_cr2);
//     sleep_ms(1);
// }
// chip 1
// lx1f pin is red
// lx2f pin is green

// chip 2
// lx1f pin is white
// lx2f pin is blue
