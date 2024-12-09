/**
 * Copyright (c) 2021 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "pio_smbus.h"

#include "pico/time.h"

const int PIO_I2C_CMD_WIDTH = 10;
const int PIO_I2C_FINAL_LSB = 9;
const int PIO_I2C_DATA_LSB = 1;
const int PIO_I2C_NAK_LSB = 0;

// Cumulative Clock Low Slave
// The slave device must not exceed this amount of time for SCLK to be low during the transfer
// If the slave holds the clock longer than this time, a timeout occurs
#define SMBUS_SEXT_TIME_MS 25
// The maximum amount of time the bus can be held low
// If the bus is low for longer than this amount of time, then all devices must reset their transmitters
#define SMBUS_TIMEOUT_MS 35
// Amount of milliseconds to act as a buffer after a timeout before giving up sending a stop condition
// Not in specification, but tunable by us to allow devicings triggering timeout some leeway to release the bus
#define SMBUS_TIMEOUT_BUFFER_MS 5
// Minimum start/stop sequence duration in microseconds for smbus transfers
// Smbus specifies 4.7us, 5us should work well for us
#define SMBUS_START_STOP_SEQ_DURATION_US 5

#define PIO_I2C_XFER_TX_ACK 0
#define PIO_I2C_XFER_REQUIRE_ACK ((1 << PIO_I2C_NAK_LSB))
#define PIO_I2C_XFER_ALLOW_NAK ((1 << PIO_I2C_NAK_LSB) | (1 << PIO_I2C_FINAL_LSB))

// ========================================
// Low Level Hardware Utility Functions
// ========================================

__force_inline static int pio_smbus_check_bus_inactive(PIO pio, uint sm) {
    // Checks the SDA and SCL pins defined for the bus and make sure they're high
    // This directly checks the IO bank, although it isn't as fast, this bypasses any weird configuration stuff that
    // might be done by I/O overrides in PIO configuration

    uint sda_pin = (pio->sm[sm].pinctrl & PIO_SM0_PINCTRL_IN_BASE_BITS) >> PIO_SM0_PINCTRL_IN_BASE_LSB;
    uint scl_pin = (sda_pin + i2c_scl_offset) % 32;
    uint sda_val =
        (iobank0_hw->io[sda_pin].status & IO_BANK0_GPIO0_STATUS_INFROMPAD_BITS) >> IO_BANK0_GPIO0_STATUS_INFROMPAD_LSB;
    uint scl_val =
        (iobank0_hw->io[scl_pin].status & IO_BANK0_GPIO0_STATUS_INFROMPAD_BITS) >> IO_BANK0_GPIO0_STATUS_INFROMPAD_LSB;

    if (!sda_val || !scl_val) {
        return PIO_SMBUS_ERR_BUS_STUCK_LOW;
    }
    else {
        return PIO_SMBUS_SUCCESS;
    }
}

__force_inline static bool pio_smbus_check_nak_error(PIO pio, uint sm) {
    return pio_interrupt_get(pio, sm);
}

__force_inline static void pio_smbus_busy_wait_us_fast(uint32_t delay_us) {
    // NOTE: Time must be < 2^31 to avoid a race condition
    uint32_t start = timer_hw->timerawl;
    while (timer_hw->timerawl - start < delay_us) {
        tight_loop_contents();
    }
}

// ========================================
// Low Level PIO Control Functions
// ========================================

static void pio_smbus_fix_after_timeout(PIO pio, uint sm) {
    // Disable SM so it can't do anything when we move it back to normal
    pio_sm_set_enabled(pio, sm, false);
    // Clear the internal state so we can park it back at the start
    pio_sm_restart(pio, sm);
    // Clear any random extra data in the FIFOs to be safe
    pio_sm_clear_fifos(pio, sm);
    // Go back to the entrypoint (the .wrap_target)
    pio_sm_exec(pio, sm,
                (pio->sm[sm].execctrl & PIO_SM0_EXECCTRL_WRAP_BOTTOM_BITS) >> PIO_SM0_EXECCTRL_WRAP_BOTTOM_LSB);
    // Now ready to re-enable for new commands
    pio_sm_set_enabled(pio, sm, true);
}

__always_inline static int pio_smbus_exec_pinset(PIO pio, uint sm, uint instr) {
    // Assumption: SM is idle
    // NOTE: instr MUST NOT STALL (See section 3.2.4 for definitions on what this can be defined on)
    pio_sm_exec(pio, sm, instr);
    // After setting time, need to delay by minimum time defined by SMBus protocol
    pio_smbus_busy_wait_us_fast(SMBUS_START_STOP_SEQ_DURATION_US);
    // This function leaves the SM idle (since instr won't stall)
    return PIO_SMBUS_SUCCESS;
}

__always_inline static int pio_smbus_wait_for_clock_high_timeout(PIO pio, uint sm, uint32_t timeout) {
    // Assumption: SM is idle
    // Wait for SCL to go high
    pio_sm_exec(pio, sm, pio_encode_wait_pin(true, i2c_scl_offset));

    // Wait for SCL to go high, or timeout (defined by SMBus specification)
    absolute_time_t bus_timeout = make_timeout_time_ms(timeout);
    while (pio_sm_is_exec_stalled(pio, sm)) {
        if (time_reached(bus_timeout)) {
            // We timed out, abort the wait
            pio_sm_exec(pio, sm, pio_encode_nop());
            return PIO_SMBUS_ERR_TIMEOUT;
        }
    }

    // This function leaves the SM idle
    return PIO_SMBUS_SUCCESS;
}

__always_inline static int pio_smbus_wait_for_clock_high(PIO pio, uint sm) {
    return pio_smbus_wait_for_clock_high_timeout(pio, sm, SMBUS_SEXT_TIME_MS);
}

static int __time_critical_func(pio_smbus_do_xfer)(PIO pio, uint sm, uint8_t txdata, uint xfer_mode) {
    // Assumption: SM is idle
    // Write the data to the transmit FIFO
    pio_sm_put(pio, sm, (xfer_mode | (((uint) txdata) << PIO_I2C_DATA_LSB)) << (32 - PIO_I2C_CMD_WIDTH));

    // Now, wait for three possible end conditions
    // 1. XFER successfully goes onto the bus
    // 2. SM enters failure state due to NAK
    // 3. We time out waiting for a clock stretch
    absolute_time_t bus_timeout = make_timeout_time_ms(SMBUS_SEXT_TIME_MS);
    while (pio_sm_is_rx_fifo_empty(pio, sm)) {
        if (time_reached(bus_timeout)) {
            pio_smbus_fix_after_timeout(pio, sm);
            return PIO_SMBUS_ERR_TIMEOUT;
        }
        // NAK error
        if (pio_smbus_check_nak_error(pio, sm)) {
            // Clear the interrupt flag, releasing the SM to go back to idle
            pio_interrupt_clear(pio, sm);
            return PIO_SMBUS_ERR_NAK;
        }
    }

    // We got data back, xfer was successful
    uint8_t rxdata = pio_sm_get(pio, sm);
    // The SM should be idle (since we push right before returning back to idle state)
    return rxdata;
}

// ========================================
// I2C Primitive Operations
// ========================================

#define forward_err(func)                                                                                              \
    do {                                                                                                               \
        int rc = func;                                                                                                 \
        if (rc)                                                                                                        \
            return rc;                                                                                                 \
    } while (0)

__force_inline static int pio_smbus_tx_byte_opt_nak(PIO pio, uint sm, uint8_t data, bool permit_nak) {
    // Assumption: SM is idle
    int ret = pio_smbus_do_xfer(pio, sm, data, (permit_nak ? PIO_I2C_XFER_ALLOW_NAK : PIO_I2C_XFER_REQUIRE_ACK));

    // Return any errors during xfer
    if (ret < 0) {
        return ret;
    }

    // Make sure the data we put onto SDA matches the data we received
    // This can detect a stuck SDA (which will still appear as an ACK)
    if (ret != data) {
        return PIO_SMBUS_ERR_ABRITRATION_LOST;
    }
    else {
        return PIO_SMBUS_SUCCESS;
    }
    // Leaves SM idle after
}

__force_inline static int pio_smbus_tx_byte(PIO pio, uint sm, uint8_t data) {
    // Same as pio_smbus_tx_byte_opt_nak, but requires that the packet be acked
    return pio_smbus_tx_byte_opt_nak(pio, sm, data, false);
}

__force_inline static int pio_smbus_rx_byte(PIO pio, uint sm, bool send_ack) {
    // Perform the xfer, using 0xFF so we let the transmitter take control of the bus
    // Assumption: SM is idle
    return pio_smbus_do_xfer(pio, sm, 0xFF, (send_ack ? PIO_I2C_XFER_TX_ACK : PIO_I2C_XFER_ALLOW_NAK));
    // Leaves SM idle after
}

static int __time_critical_func(pio_smbus_do_start)(PIO pio, uint sm) {
    // Sends a start condition onto the i2c bus, and gets it into a state for bytes to be xferred
    // Assumption: SM is idle, bus is in inactive state
    forward_err(pio_smbus_check_bus_inactive(pio, sm));
    forward_err(pio_smbus_exec_pinset(pio, sm, set_scl_sda_program_instructions[I2C_SC1_SD0]));
    forward_err(pio_smbus_exec_pinset(pio, sm, set_scl_sda_program_instructions[I2C_SC0_SD0]));
    // Leaves SM idle after
    return PIO_SMBUS_SUCCESS;
}

static int __time_critical_func(pio_smbus_do_repstart)(PIO pio, uint sm) {
    // Sends a repeated start condition onto the i2c bus
    // Assumption: SM is idle
    forward_err(pio_smbus_exec_pinset(pio, sm, set_scl_sda_program_instructions[I2C_SC0_SD1]));
    forward_err(pio_smbus_exec_pinset(pio, sm, set_scl_sda_program_instructions[I2C_SC1_SD1]));
    forward_err(pio_smbus_wait_for_clock_high(pio, sm));
    forward_err(pio_smbus_exec_pinset(pio, sm, set_scl_sda_program_instructions[I2C_SC1_SD0]));
    forward_err(pio_smbus_exec_pinset(pio, sm, set_scl_sda_program_instructions[I2C_SC0_SD0]));
    // Leaves SM idle after
    return PIO_SMBUS_SUCCESS;
}

static int __time_critical_func(pio_smbus_do_stop)(PIO pio, uint sm) {
    // Sends a stop condition onto the i2c bus (this will require a *start* before more bytes can be xferred)
    // Assumption: SM is idle
    forward_err(pio_smbus_exec_pinset(pio, sm, set_scl_sda_program_instructions[I2C_SC0_SD0]));
    forward_err(pio_smbus_exec_pinset(pio, sm, set_scl_sda_program_instructions[I2C_SC1_SD0]));
    forward_err(pio_smbus_wait_for_clock_high(pio, sm));
    forward_err(pio_smbus_exec_pinset(pio, sm, set_scl_sda_program_instructions[I2C_SC1_SD1]));
    // Leaves SM idle after, and bus in inactive state
    return PIO_SMBUS_SUCCESS;
}

static void pio_smbus_cleanup_failed_operation(PIO pio, uint sm, int err) {
    uint32_t timeout = SMBUS_TIMEOUT_MS + SMBUS_TIMEOUT_BUFFER_MS;
    if (err == PIO_SMBUS_ERR_TIMEOUT) {
        // We already saw a timeout, we can subtract the SEXT time from our check
        timeout -= SMBUS_SEXT_TIME_MS;
    }

    // Ignore all errors, we'll try our best to put a stop condition on the bus, but if that isn't possible, it'll be
    // caught on the next transmission
    pio_smbus_exec_pinset(pio, sm, set_scl_sda_program_instructions[I2C_SC0_SD0]);
    pio_smbus_exec_pinset(pio, sm, set_scl_sda_program_instructions[I2C_SC1_SD0]);
    pio_smbus_wait_for_clock_high_timeout(pio, sm, timeout);
    pio_smbus_exec_pinset(pio, sm, set_scl_sda_program_instructions[I2C_SC1_SD1]);
}

// ========================================
// SMBus Standard Transfers (Exported)
// ========================================

#define check_err(func)                                                                                                \
    do {                                                                                                               \
        err = func;                                                                                                    \
        if (err)                                                                                                       \
            goto fail;                                                                                                 \
    } while (0)

#define check_err_ovrd_nak(func, new_nak_err)                                                                          \
    do {                                                                                                               \
        err = func;                                                                                                    \
        if (err == PIO_SMBUS_ERR_NAK)                                                                                  \
            err = new_nak_err;                                                                                         \
        if (err)                                                                                                       \
            goto fail;                                                                                                 \
    } while (0)

#define check_rx(out, func)                                                                                            \
    do {                                                                                                               \
        err = func;                                                                                                    \
        if (err < 0)                                                                                                   \
            goto fail;                                                                                                 \
        (out) = err;                                                                                                   \
    } while (0)

int __time_critical_func(pio_smbus_word_read)(PIO pio, uint sm, uint8_t addr, uint8_t cmd) {
    int err;

    // Perform i2c write of single byte to send the command to read
    // We don't allow NAKs
    check_err(pio_smbus_do_start(pio, sm));
    check_err_ovrd_nak(pio_smbus_tx_byte(pio, sm, (addr << 1)), PIO_SMBUS_ERR_ADDR_NAK);
    check_err(pio_smbus_tx_byte(pio, sm, cmd));

    uint16_t word;
    uint8_t data;

    // Now switch to read to read the word out
    check_err(pio_smbus_do_repstart(pio, sm));
    check_err_ovrd_nak(pio_smbus_tx_byte(pio, sm, (addr << 1) | 1u), PIO_SMBUS_ERR_ADDR_RESTART_NAK);
    // Receive lower byte
    check_rx(word, pio_smbus_rx_byte(pio, sm, true));
    // Receive upper byte
    check_rx(data, pio_smbus_rx_byte(pio, sm, false));  // Don't ack last byte
    word |= ((uint16_t) data) << 8;
    check_err(pio_smbus_do_stop(pio, sm));

    return word;

fail:
    pio_smbus_cleanup_failed_operation(pio, sm, err);
    return err;
}

int __time_critical_func(pio_smbus_word_write)(PIO pio, uint sm, uint8_t addr, uint8_t cmd, uint16_t data) {
    int err;

    // Perform i2c write of single byte to send the command to read
    // We don't allow NAKs
    check_err(pio_smbus_do_start(pio, sm));
    check_err_ovrd_nak(pio_smbus_tx_byte(pio, sm, (addr << 1)), PIO_SMBUS_ERR_ADDR_NAK);
    check_err(pio_smbus_tx_byte(pio, sm, cmd));
    check_err(pio_smbus_tx_byte(pio, sm, data & 0xFF));
    check_err(pio_smbus_tx_byte(pio, sm, data >> 8));
    check_err(pio_smbus_do_stop(pio, sm));

    return PIO_SMBUS_SUCCESS;

fail:
    pio_smbus_cleanup_failed_operation(pio, sm, err);
    return err;
}

int __time_critical_func(pio_smbus_block_read)(PIO pio, uint sm, uint8_t addr, uint8_t cmd, uint8_t *rxbuf,
                                               uint max_len) {
    int err;

    // Perform i2c write of single byte to send the command to read
    // We don't allow NAKs
    check_err(pio_smbus_do_start(pio, sm));
    check_err_ovrd_nak(pio_smbus_tx_byte(pio, sm, (addr << 1)), PIO_SMBUS_ERR_ADDR_NAK);
    check_err(pio_smbus_tx_byte(pio, sm, cmd));

    uint8_t len;

    // Now switch to read to read the word out
    check_err(pio_smbus_do_repstart(pio, sm));
    check_err_ovrd_nak(pio_smbus_tx_byte(pio, sm, (addr << 1) | 1u), PIO_SMBUS_ERR_ADDR_RESTART_NAK);

    // Receive length of block to read
    check_rx(len, pio_smbus_rx_byte(pio, sm, true));
    if (len > max_len) {
        // We can't fit all the bytes into our buffer
        // Receive and NAK so we notify the receiver we're stopping
        // No need to check for error, since we're gonna error anyways
        pio_smbus_rx_byte(pio, sm, false);
        err = PIO_SMBUS_ERR_BUF_TOO_SMALL;
        goto fail;
    }

    // Receive all the bytes into the buffer
    for (size_t i = 0; i < len; i++) {
        check_rx(rxbuf[i], pio_smbus_rx_byte(pio, sm, i + 1 != len));
    }

    // Finish the transfer
    check_err(pio_smbus_do_stop(pio, sm));

    // Return number of received bytes
    return len;

fail:
    pio_smbus_cleanup_failed_operation(pio, sm, err);
    return err;
}

int __time_critical_func(pio_smbus_block_write)(PIO pio, uint sm, uint8_t addr, uint8_t cmd, uint8_t *txbuf, uint len) {
    int err;

    if (len > UINT8_MAX) {
        return PIO_SMBUS_ERR_BUF_TOO_LARGE;
    }

    // Perform i2c write of the command followed by the written bytes
    // We don't allow NAKs
    check_err(pio_smbus_do_start(pio, sm));
    check_err_ovrd_nak(pio_smbus_tx_byte(pio, sm, (addr << 1)), PIO_SMBUS_ERR_ADDR_NAK);
    check_err(pio_smbus_tx_byte(pio, sm, cmd));
    check_err(pio_smbus_tx_byte(pio, sm, len));
    for (size_t i = 0; i < len; i++) {
        check_err(pio_smbus_tx_byte(pio, sm, txbuf[i]));
    }
    check_err(pio_smbus_do_stop(pio, sm));

    return PIO_SMBUS_SUCCESS;

fail:
    pio_smbus_cleanup_failed_operation(pio, sm, err);
    return err;
}

// ========================================
// SMBus PEC Transfers (Exported)
// ========================================

// CRC 8 lookup table
// Not const so it's in RAM so we don't get hit by cache latencies during time sensitive operations
static uint8_t crc8_table[] = {
    0x00, 0x07, 0x0e, 0x09, 0x1c, 0x1b, 0x12, 0x15, 0x38, 0x3f, 0x36, 0x31, 0x24, 0x23, 0x2a, 0x2d, 0x70, 0x77, 0x7e,
    0x79, 0x6c, 0x6b, 0x62, 0x65, 0x48, 0x4f, 0x46, 0x41, 0x54, 0x53, 0x5a, 0x5d, 0xe0, 0xe7, 0xee, 0xe9, 0xfc, 0xfb,
    0xf2, 0xf5, 0xd8, 0xdf, 0xd6, 0xd1, 0xc4, 0xc3, 0xca, 0xcd, 0x90, 0x97, 0x9e, 0x99, 0x8c, 0x8b, 0x82, 0x85, 0xa8,
    0xaf, 0xa6, 0xa1, 0xb4, 0xb3, 0xba, 0xbd, 0xc7, 0xc0, 0xc9, 0xce, 0xdb, 0xdc, 0xd5, 0xd2, 0xff, 0xf8, 0xf1, 0xf6,
    0xe3, 0xe4, 0xed, 0xea, 0xb7, 0xb0, 0xb9, 0xbe, 0xab, 0xac, 0xa5, 0xa2, 0x8f, 0x88, 0x81, 0x86, 0x93, 0x94, 0x9d,
    0x9a, 0x27, 0x20, 0x29, 0x2e, 0x3b, 0x3c, 0x35, 0x32, 0x1f, 0x18, 0x11, 0x16, 0x03, 0x04, 0x0d, 0x0a, 0x57, 0x50,
    0x59, 0x5e, 0x4b, 0x4c, 0x45, 0x42, 0x6f, 0x68, 0x61, 0x66, 0x73, 0x74, 0x7d, 0x7a, 0x89, 0x8e, 0x87, 0x80, 0x95,
    0x92, 0x9b, 0x9c, 0xb1, 0xb6, 0xbf, 0xb8, 0xad, 0xaa, 0xa3, 0xa4, 0xf9, 0xfe, 0xf7, 0xf0, 0xe5, 0xe2, 0xeb, 0xec,
    0xc1, 0xc6, 0xcf, 0xc8, 0xdd, 0xda, 0xd3, 0xd4, 0x69, 0x6e, 0x67, 0x60, 0x75, 0x72, 0x7b, 0x7c, 0x51, 0x56, 0x5f,
    0x58, 0x4d, 0x4a, 0x43, 0x44, 0x19, 0x1e, 0x17, 0x10, 0x05, 0x02, 0x0b, 0x0c, 0x21, 0x26, 0x2f, 0x28, 0x3d, 0x3a,
    0x33, 0x34, 0x4e, 0x49, 0x40, 0x47, 0x52, 0x55, 0x5c, 0x5b, 0x76, 0x71, 0x78, 0x7f, 0x6a, 0x6d, 0x64, 0x63, 0x3e,
    0x39, 0x30, 0x37, 0x22, 0x25, 0x2c, 0x2b, 0x06, 0x01, 0x08, 0x0f, 0x1a, 0x1d, 0x14, 0x13, 0xae, 0xa9, 0xa0, 0xa7,
    0xb2, 0xb5, 0xbc, 0xbb, 0x96, 0x91, 0x98, 0x9f, 0x8a, 0x8d, 0x84, 0x83, 0xde, 0xd9, 0xd0, 0xd7, 0xc2, 0xc5, 0xcc,
    0xcb, 0xe6, 0xe1, 0xe8, 0xef, 0xfa, 0xfd, 0xf4, 0xf3
};

#define crc8_update(var, val) (var) = crc8_table[((uint8_t) (var)) ^ ((uint8_t) (val))]
#define pio_smbus_check_tx_byte_pec(pio, sm, crc_var, data)                                                            \
    do {                                                                                                               \
        check_err(pio_smbus_tx_byte(pio, sm, data));                                                                   \
        crc8_update(crc_var, data);                                                                                    \
    } while (0)
#define pio_smbus_check_tx_byte_pec_ovrd_nak(pio, sm, crc_var, data, new_nak_err)                                      \
    do {                                                                                                               \
        check_err_ovrd_nak(pio_smbus_tx_byte(pio, sm, data), new_nak_err);                                             \
        crc8_update(crc_var, data);                                                                                    \
    } while (0)
#define pio_smbus_check_rx_byte_pec(out, pio, sm, crc_var)                                                             \
    do {                                                                                                               \
        check_rx(out, pio_smbus_rx_byte(pio, sm, true));                                                               \
        crc8_update(crc_var, out);                                                                                     \
    } while (0)

int __time_critical_func(pio_smbus_word_read_pec)(PIO pio, uint sm, uint8_t addr, uint8_t cmd) {
    int err;
    uint8_t crc_calc = 0;

    // Perform i2c write of single byte to send the command to read
    // We don't allow NAKs
    check_err(pio_smbus_do_start(pio, sm));
    pio_smbus_check_tx_byte_pec_ovrd_nak(pio, sm, crc_calc, (addr << 1), PIO_SMBUS_ERR_ADDR_NAK);
    pio_smbus_check_tx_byte_pec(pio, sm, crc_calc, cmd);

    uint16_t word;
    uint8_t data;

    // Now switch to read to read the word out
    check_err(pio_smbus_do_repstart(pio, sm));
    pio_smbus_check_tx_byte_pec_ovrd_nak(pio, sm, crc_calc, (addr << 1) | 1u, PIO_SMBUS_ERR_ADDR_RESTART_NAK);
    // Receive lower byte
    pio_smbus_check_rx_byte_pec(word, pio, sm, crc_calc);
    // Receive upper byte
    pio_smbus_check_rx_byte_pec(data, pio, sm, crc_calc);
    word |= ((uint16_t) data) << 8;

    // Now receive the checksum (always NAK as master)
    uint8_t crc_rx;
    check_rx(crc_rx, pio_smbus_rx_byte(pio, sm, false));

    // Send stop
    check_err(pio_smbus_do_stop(pio, sm));

    // Report error if the checksum doesn't match the expected
    if (crc_rx != crc_calc) {
        return PIO_SMBUS_ERR_BAD_CHECKSUM;
    }

    return word;

fail:
    pio_smbus_cleanup_failed_operation(pio, sm, err);
    return err;
}

int __time_critical_func(pio_smbus_word_write_pec)(PIO pio, uint sm, uint8_t addr, uint8_t cmd, uint16_t data) {
    int err;
    uint8_t crc_calc = 0;

    // Perform i2c write of single byte to send the command to read
    // We don't allow NAKs
    check_err(pio_smbus_do_start(pio, sm));
    pio_smbus_check_tx_byte_pec_ovrd_nak(pio, sm, crc_calc, (addr << 1), PIO_SMBUS_ERR_ADDR_NAK);
    pio_smbus_check_tx_byte_pec(pio, sm, crc_calc, cmd);
    pio_smbus_check_tx_byte_pec(pio, sm, crc_calc, data & 0xFF);
    pio_smbus_check_tx_byte_pec(pio, sm, crc_calc, data >> 8);

    // Now transmit the PEC byte, but report NAK errors as checksum error (since the slave NAKs bad checksums)
    check_err_ovrd_nak(pio_smbus_tx_byte(pio, sm, crc_calc), PIO_SMBUS_ERR_BAD_CHECKSUM);
    check_err(pio_smbus_do_stop(pio, sm));

    return PIO_SMBUS_SUCCESS;

fail:
    pio_smbus_cleanup_failed_operation(pio, sm, err);
    return err;
}

int __time_critical_func(pio_smbus_block_read_pec)(PIO pio, uint sm, uint8_t addr, uint8_t cmd, uint8_t *rxbuf,
                                                   uint max_len) {
    int err;
    uint8_t crc_calc = 0;

    // Perform i2c write of single byte to send the command to read
    // We don't allow NAKs
    check_err(pio_smbus_do_start(pio, sm));
    pio_smbus_check_tx_byte_pec_ovrd_nak(pio, sm, crc_calc, (addr << 1), PIO_SMBUS_ERR_ADDR_NAK);
    pio_smbus_check_tx_byte_pec(pio, sm, crc_calc, cmd);

    uint8_t len;

    // Now switch to read to read the word out
    check_err(pio_smbus_do_repstart(pio, sm));
    pio_smbus_check_tx_byte_pec_ovrd_nak(pio, sm, crc_calc, (addr << 1) | 1u, PIO_SMBUS_ERR_ADDR_RESTART_NAK);

    // Receive length of block to read
    pio_smbus_check_rx_byte_pec(len, pio, sm, crc_calc);
    if (len > max_len) {
        // We can't fit all the bytes into our buffer
        // Receive and NAK so we notify the receiver we're stopping
        // No need to check for error, since we're gonna error anyways
        pio_smbus_rx_byte(pio, sm, false);
        err = PIO_SMBUS_ERR_BUF_TOO_SMALL;
        goto fail;
    }

    // Receive all the bytes into the buffer
    for (size_t i = 0; i < len; i++) {
        pio_smbus_check_rx_byte_pec(rxbuf[i], pio, sm, crc_calc);
    }

    // Now receive the checksum (always NAK as master)
    uint8_t crc_rx;
    check_rx(crc_rx, pio_smbus_rx_byte(pio, sm, false));

    // Finish the transfer
    check_err(pio_smbus_do_stop(pio, sm));

    // Report error if the checksum doesn't match
    if (crc_rx != crc_calc) {
        return PIO_SMBUS_ERR_BAD_CHECKSUM;
    }

    // Return number of received bytes
    return len;

fail:
    pio_smbus_cleanup_failed_operation(pio, sm, err);
    return err;
}

int __time_critical_func(pio_smbus_block_write_pec)(PIO pio, uint sm, uint8_t addr, uint8_t cmd, uint8_t *txbuf,
                                                    uint len) {
    int err;
    uint8_t crc_calc = 0;

    if (len > UINT8_MAX) {
        return PIO_SMBUS_ERR_BUF_TOO_LARGE;
    }

    // Perform i2c write of the command followed by the written bytes
    // We don't allow NAKs
    check_err(pio_smbus_do_start(pio, sm));
    pio_smbus_check_tx_byte_pec_ovrd_nak(pio, sm, crc_calc, (addr << 1), PIO_SMBUS_ERR_ADDR_NAK);
    pio_smbus_check_tx_byte_pec(pio, sm, crc_calc, cmd);
    pio_smbus_check_tx_byte_pec(pio, sm, crc_calc, len);
    for (size_t i = 0; i < len; i++) {
        pio_smbus_check_tx_byte_pec(pio, sm, crc_calc, txbuf[i]);
    }

    // Now transmit the PEC byte, but report NAK errors as checksum error (since the slave NAKs bad checksums)
    check_err_ovrd_nak(pio_smbus_tx_byte(pio, sm, crc_calc), PIO_SMBUS_ERR_BAD_CHECKSUM);
    check_err(pio_smbus_do_stop(pio, sm));

    return PIO_SMBUS_SUCCESS;

fail:
    pio_smbus_cleanup_failed_operation(pio, sm, err);
    return err;
}
