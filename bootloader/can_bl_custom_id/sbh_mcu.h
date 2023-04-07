#ifndef CAN_BL_CUSTOM_ID__SBH_MCU_H
#define CAN_BL_CUSTOM_ID__SBH_MCU_H

#include <stdbool.h>
#include <stdint.h>
#include "hardware/gpio.h"
#include "hardware/timer.h"

// So we need to determine the client id based on whether its the port or stbd pack
// But due to the i2c lines being flipped, we can't rely on the i2c peripheral
// And we definitely don't have enough room for PIO soft i2c
// But we *CAN* bitbang it, since we've got 125 MHz of pure Cortex-M0 _SPEED_
// So now I'm going to implement an i2c bitbang protocol
// Good news is that it should still work even if the ordering does get fixed in a future rev, since bitbanging doesn't care about pins
// And if it works, like we don't need all the error handling capabilities of the actual i2c block anyways
// So screw it

#define I2C_TIMEOUT_MS 30
#define I2C_CLK_RATE_KHZ 20

// ========================================
// Low-Level Bitbanging
// ========================================

bool bitbang_i2c_wait_for_clock_high(void) {
    uint64_t timeout = time_us_64() + (I2C_TIMEOUT_MS * 1000);
    while (time_us_64() < timeout) {
        if (gpio_get(BMS_SCL_PIN)) {
            return true;
        }
    }
    return false;
}

static inline void bitbang_i2c_delay_quarter_clock(void) {
    busy_wait_us((1000 / I2C_CLK_RATE_KHZ) / 4);
}

static inline void bitbang_i2c_delay_half_clock(void) {
    busy_wait_us((1000 / I2C_CLK_RATE_KHZ) / 2);
}

static inline void bitbang_i2c_set_sda(int high) {
    gpio_set_dir(BMS_SDA_PIN, (high ? 0 : 1));
}

static inline void bitbang_i2c_release_sda(void) {
    gpio_set_dir(BMS_SDA_PIN, 0);
}

static inline bool bitbang_i2c_read_sda(void) {
    return (gpio_get(BMS_SDA_PIN) ? true : false);
}

static inline void bitbang_i2c_drive_scl(void) {
    gpio_set_dir(BMS_SCL_PIN, 1);
}

static inline bool bitbang_i2c_release_scl(void) {
    gpio_set_dir(BMS_SCL_PIN, 0);
    return bitbang_i2c_wait_for_clock_high();
}

// ========================================
// I2C Single-bit Bus States
// ========================================

static bool bitbang_i2c_send_bit(bool bit) {
    bitbang_i2c_set_sda(bit);
    bitbang_i2c_delay_quarter_clock();
    if (!bitbang_i2c_release_scl())  return false;
    bitbang_i2c_delay_half_clock();
    bitbang_i2c_drive_scl();
    bitbang_i2c_delay_quarter_clock();

    return true;
}

static bool bitbang_i2c_get_bit(bool *bit) {
    bitbang_i2c_release_sda();
    bitbang_i2c_delay_quarter_clock();
    if (!bitbang_i2c_release_scl()) {
        return false;
    }
    bitbang_i2c_delay_quarter_clock();
    *bit = (bitbang_i2c_read_sda() ? false : true);
    bitbang_i2c_delay_quarter_clock();
    bitbang_i2c_drive_scl();
    bitbang_i2c_delay_quarter_clock();

    return true;
}

static void bitbang_i2c_send_start(void) {
    bitbang_i2c_set_sda(false);
    bitbang_i2c_delay_half_clock();
    bitbang_i2c_drive_scl();
    bitbang_i2c_delay_quarter_clock();
}

static bool bitbang_i2c_send_stop(void) {
    bitbang_i2c_set_sda(0);
    bitbang_i2c_delay_quarter_clock();
    if (!bitbang_i2c_release_scl()) {
        return false;
    }
    bitbang_i2c_delay_half_clock();
    bitbang_i2c_release_sda();

    return true;
}

// ========================================
// I2C Byte-oriented Functions
// ========================================

static bool bitbang_i2c_send_byte(uint8_t byte) {
    for (int i = 7; i >= 0; i--) {
        if (!bitbang_i2c_send_bit((byte >> i) & 1)) return false;
    }

    return true;
}

static bool bitbang_i2c_get_byte(uint8_t *byte_out) {
    uint8_t byte = 0;
    for (int i = 7; i >= 0; i--) {
        bool val;
        if (!bitbang_i2c_get_bit(&val)) return false;
        if (val) byte |= (1<<i);
    }
    *byte_out = byte;

    return true;
}

// ========================================
// I2C Transfer-oriented Functions
// ========================================

static bool bitbang_i2c_transfer_byte(uint8_t id, uint8_t byte) {
    // Wait for clock first
    if (!bitbang_i2c_wait_for_clock_high()) return false;

    bool got_ack = false;

    // Start condition
    bitbang_i2c_send_start();

    // Transmit ID
    id <<= 1;   // LSB is direction (0 for TX)
    if (!bitbang_i2c_send_byte(id)) return false;
    if (!bitbang_i2c_get_bit(&got_ack)) return false;
    if (!got_ack) goto stop;

    // Single data transmission
    bool unused;    // Don't care about NACK on last byte
    if (!bitbang_i2c_send_byte(byte)) return false;
    if (!bitbang_i2c_get_bit(&unused)) return false;

stop:
    // Stop transmission
    if (!bitbang_i2c_send_stop()) return false;

    return got_ack;
}

static bool bitbang_i2c_recv_word(uint8_t id, uint16_t *word) {
    // Wait for clock first
    if (!bitbang_i2c_wait_for_clock_high()) return false;

    bool got_ack = false;

    // Start condition
    bitbang_i2c_send_start();

    // Transmit ID
    id <<= 1;   // LSB is direction (1 for RX)
    id |= 1;
    if (!bitbang_i2c_send_byte(id)) return false;
    if (!bitbang_i2c_get_bit(&got_ack)) return false;
    if (!got_ack) goto stop;

    // Two-byte data rx
    uint8_t byte0, byte1;
    if (!bitbang_i2c_get_byte(&byte0)) return false;
    if (!bitbang_i2c_send_bit(0)) return false; // ACK reception

    if (!bitbang_i2c_get_byte(&byte1)) return false;
    if (!bitbang_i2c_send_bit(1)) return false; // NACK last byte

    *word = ((byte1 << 8) | byte0);

stop:
    // Stop transmission
    if (!bitbang_i2c_send_stop()) return false;

    return got_ack;
}

// ========================================
// Exported Function
// ========================================

static bool bl_board_get_client_id(int *client_id) {
    gpio_init(BMS_SCL_PIN);
    gpio_init(BMS_SDA_PIN);
    gpio_pull_up(BMS_SCL_PIN);
    gpio_pull_up(BMS_SDA_PIN);
    gpio_put(BMS_SCL_PIN, 0);
    gpio_put(BMS_SDA_PIN, 0);

    // The ID for the BMS chip
    const uint8_t bms_i2c_id = 0x0B;

    // The bit mask for the side detection pin
    const uint8_t bms_side_detect_pin_mask = (1 << 3);

    // Try 3 times
    for (int i = 0; i < 3; i++) {
        // Put bus in normal state
        gpio_set_dir(BMS_SCL_PIN, 0);
        gpio_set_dir(BMS_SDA_PIN, 0);
        busy_wait_ms(5);    // Give time for things to settle before starting another transaction

        // Attempt to send BMS GPIORead() command
        if (!bitbang_i2c_transfer_byte(bms_i2c_id, 0x48)) {
            continue;
        }

        uint16_t gpio_state = 0;
        if (!bitbang_i2c_recv_word(bms_i2c_id, &gpio_state)) {
            continue;
        }

        if (gpio_state & bms_side_detect_pin_mask) {
            // TODO: Ensure this tracks with the actual assignment on the vehicle
            *client_id = CAN_BUS_PORT_CLIENT_ID;
        } else {
            *client_id = CAN_BUS_STBD_CLIENT_ID;
        }

        // Clean up at end in case lines got stuck for whatever reason
        gpio_set_dir(BMS_SCL_PIN, 0);
        gpio_set_dir(BMS_SDA_PIN, 0);
        return true;
    }

    // Clean up at end in case lines got stuck for whatever reason
    gpio_set_dir(BMS_SCL_PIN, 0);
    gpio_set_dir(BMS_SDA_PIN, 0);
    return false;
}


#endif
