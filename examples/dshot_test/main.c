#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/pio.h"
#include "hardware/irq.h"
#include "hardware/structs/ioqspi.h"
#include "hardware/sync.h"

#include "titan/version.h"

#include "bidir_dshot.pio.h"

#ifndef STATUS_LEDR_PIN
#define STATUS_LEDR_PIN LED_RGB_R_PIN
#endif

#ifndef STATUS_LEDG_PIN
#define STATUS_LEDG_PIN LED_RGB_G_PIN
#endif

#ifndef STATUS_LEDB_PIN
#define STATUS_LEDB_PIN LED_RGB_B_PIN
#endif

#ifndef ESC1_PWM_PIN
#define ESC1_PWM_PIN THRUSTER_5_PIN
#endif

#ifndef ESC2_PWM_PIN
#define ESC2_PWM_PIN THRUSTER_6_PIN
#endif

#ifndef ESC3_PWM_PIN
#define ESC3_PWM_PIN THRUSTER_7_PIN
#endif

#ifndef ESC4_PWM_PIN
#define ESC4_PWM_PIN THRUSTER_8_PIN
#endif

bool __no_inline_not_in_flash_func(get_bootsel_button)() {
    const uint CS_PIN_INDEX = 1;

    // Must disable interrupts, as interrupt handlers may be in flash, and we
    // are about to temporarily disable flash access!
    uint32_t flags = save_and_disable_interrupts();

    // Set chip select to Hi-Z
    hw_write_masked(&ioqspi_hw->io[CS_PIN_INDEX].ctrl,
                    GPIO_OVERRIDE_LOW << IO_QSPI_GPIO_QSPI_SS_CTRL_OEOVER_LSB,
                    IO_QSPI_GPIO_QSPI_SS_CTRL_OEOVER_BITS);

    // Note we can't call into any sleep functions in flash right now
    for (volatile int i = 0; i < 1000; ++i);

    // The HI GPIO registers in SIO can observe and control the 6 QSPI pins.
    // Note the button pulls the pin *low* when pressed.
    bool button_state = !(sio_hw->gpio_hi_in & (1u << CS_PIN_INDEX));

    // Need to restore the state of chip select, else we are going to have a
    // bad time when we return to code in flash!
    hw_write_masked(&ioqspi_hw->io[CS_PIN_INDEX].ctrl,
                    GPIO_OVERRIDE_NORMAL << IO_QSPI_GPIO_QSPI_SS_CTRL_OEOVER_LSB,
                    IO_QSPI_GPIO_QSPI_SS_CTRL_OEOVER_BITS);

    restore_interrupts(flags);

    return button_state;
}

void init_leds(void) {
    const uint led_pins = ((1<<STATUS_LEDR_PIN) | (1<<STATUS_LEDG_PIN) | (1<<STATUS_LEDB_PIN));
    gpio_init_mask(led_pins);
    gpio_set_mask(led_pins);
    gpio_set_dir_out_masked(led_pins);
}

#define DSHOT_PIO pio0
#define DSHOT_RATE 300


int8_t gcr_lookup[0x20] = {
    -1,  -1,  -1,  -1,  // 0x00-0x03
    -1,  -1,  -1,  -1,  // 0x04-0x07
    -1, 0x9, 0xA, 0xB,  // 0x08-0x0B
    -1, 0xD, 0xE, 0xF,  // 0x0C-0x0F
    -1,  -1, 0x2, 0x3,  // 0x10-0x13
    -1, 0x5, 0x6, 0x7,  // 0x14-0x17
    -1, 0x0, 0x8, 0x1,  // 0x18-0x1B
    -1, 0x4, 0xC,  -1,  // 0x1C-0x1F
};

int __time_critical_func(decode_erpm_period)(uint32_t packet) {
    if (packet == 0) {
        return -1;
    }

    uint32_t gcr = (packet ^ (packet >> 1));

    int8_t crc_nibble = gcr_lookup[gcr & 0x1F];
    int8_t low_nibble = gcr_lookup[(gcr >> 5) & 0x1F];
    int8_t mid_nibble = gcr_lookup[(gcr >> 10) & 0x1F];
    int8_t high_nibble = gcr_lookup[(gcr >> 15) & 0x1F];

    if (crc_nibble < 0 || low_nibble < 0 || mid_nibble < 0 || high_nibble < 0) {
        return -1;
    }

    uint32_t calculated_crc = (~(low_nibble ^ mid_nibble ^ high_nibble)) & 0xF;
    if (calculated_crc != crc_nibble) {
        return -1;
    }

    uint32_t telem_data = (high_nibble << 8) | (mid_nibble << 4) | low_nibble;
    return (telem_data & 0x1FF) << (telem_data >> 9);
}

void __time_critical_func(telem_cb)(void) {
    for (int i = 0; i < 4; i++) {
        if (pio_sm_get_rx_fifo_level(DSHOT_PIO, i) > 0) {
            int period_us = decode_erpm_period(pio_sm_get_blocking(DSHOT_PIO, i));
            if (period_us >= 0) {
                uint32_t rpm = 0;
                // Check if reporting max period (no RPM)
                if (period_us != (0x1FF << 0x7) && period_us != 0) {
                    uint32_t erpm = (60 * 1000000) / period_us;

                    // Found on forum: https://discuss.bluerobotics.com/t/t200-thruster-questions-poles-max-voltage-e-bike-controller/2442/2
                    const int num_pole_pairs = 7;
                    uint32_t rpm = erpm / num_pole_pairs;
                    //printf("RPM %d: %d\n", i+1, rpm);
                }
                // TODO: Actually store the value somewhere
            }
        }
    }
}

void dshot_send_command(uint thruster, uint throttle, bool request_telemetry) {
    uint cmd = (throttle & 0x7FF);
    cmd <<= 1;
    if (request_telemetry) {
        cmd |= 1;
    }

    uint32_t crc = (~(cmd ^ (cmd >> 4) ^ (cmd >> 8))) & 0x0F;
    cmd <<= 4;
    cmd |= crc;

    pio_sm_put_blocking(DSHOT_PIO, thruster, (~cmd) << 16);
}

int main() {
    stdio_init_all();
    init_leds();
    sleep_ms(1000);
    printf("%s\n", FULL_BUILD_TAG);

    uint offset = pio_add_program(DSHOT_PIO, &bidir_dshot_program);
    bidir_dshot_program_init(DSHOT_PIO, 0, offset, DSHOT_RATE, ESC1_PWM_PIN);
    bidir_dshot_program_init(DSHOT_PIO, 1, offset, DSHOT_RATE, ESC2_PWM_PIN);
    bidir_dshot_program_init(DSHOT_PIO, 2, offset, DSHOT_RATE, ESC3_PWM_PIN);
    bidir_dshot_program_init(DSHOT_PIO, 3, offset, DSHOT_RATE, ESC4_PWM_PIN);
    pio_set_irq0_source_mask_enabled(DSHOT_PIO,
        PIO_INTR_SM0_RXNEMPTY_BITS | PIO_INTR_SM1_RXNEMPTY_BITS |
        PIO_INTR_SM2_RXNEMPTY_BITS | PIO_INTR_SM3_RXNEMPTY_BITS,
        true);

    irq_set_exclusive_handler(PIO0_IRQ_0, telem_cb);
    irq_set_enabled(PIO0_IRQ_0, true);

    bool sent = false;
    absolute_time_t send_packet = make_timeout_time_ms(10000);

    while (true) {
        gpio_put(STATUS_LEDR_PIN, sent);
        gpio_put(STATUS_LEDG_PIN, !sent);

        if (time_reached(send_packet) && !sent) {
            puts("Sending!");
            sent = true;
            dshot_send_command(2, 4, true);
            sleep_ms(500);
        }

        if (get_bootsel_button()){
            if (sent)
                dshot_send_command(2, 1048 + 120 - 48, false);
            else
                dshot_send_command(2, 120, false);
        } else {
            dshot_send_command(2, 0, false);
        }
        sleep_ms(1);
    }
    return 0;
}