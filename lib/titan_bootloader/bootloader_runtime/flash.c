/*
 * Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "hardware/flash.h"

#include "hardware/structs/ioqspi.h"
#include "hardware/structs/ssi.h"
#include "pico/bootrom.h"

#define FLASH_BLOCK_ERASE_CMD 0xd8

// Standard RUID instruction: 4Bh command prefix, 32 dummy bits, 64 data bits.
#define FLASH_RUID_CMD 0x4b
#define FLASH_RUID_DUMMY_BYTES 4
#define FLASH_RUID_DATA_BYTES 8
#define FLASH_RUID_TOTAL_BYTES (1 + FLASH_RUID_DUMMY_BYTES + FLASH_RUID_DATA_BYTES)

//-----------------------------------------------------------------------------
// Actual flash programming shims (work whether or not PICO_NO_FLASH==1)

void __no_inline_not_in_flash_func(flash_range_erase)(uint32_t flash_offs, size_t count) {
#ifdef PICO_FLASH_SIZE_BYTES
    hard_assert(flash_offs + count <= PICO_FLASH_SIZE_BYTES);
#endif
    invalid_params_if(FLASH, flash_offs & (FLASH_SECTOR_SIZE - 1));
    invalid_params_if(FLASH, count & (FLASH_SECTOR_SIZE - 1));
    rom_connect_internal_flash_fn connect_internal_flash =
        (rom_connect_internal_flash_fn) rom_func_lookup_inline(ROM_FUNC_CONNECT_INTERNAL_FLASH);
    rom_flash_exit_xip_fn flash_exit_xip = (rom_flash_exit_xip_fn) rom_func_lookup_inline(ROM_FUNC_FLASH_EXIT_XIP);
    rom_flash_range_erase_fn flash_range_erase =
        (rom_flash_range_erase_fn) rom_func_lookup_inline(ROM_FUNC_FLASH_RANGE_ERASE);
    assert(flash_exit_xip && connect_internal_flash && flash_range_erase);

    // No flash accesses after this point
    __compiler_memory_barrier();

    connect_internal_flash();
    flash_exit_xip();
    flash_range_erase(flash_offs, count, FLASH_BLOCK_SIZE, FLASH_BLOCK_ERASE_CMD);
}

void __no_inline_not_in_flash_func(flash_range_program)(uint32_t flash_offs, const uint8_t *data, size_t count) {
#ifdef PICO_FLASH_SIZE_BYTES
    hard_assert(flash_offs + count <= PICO_FLASH_SIZE_BYTES);
#endif
    invalid_params_if(FLASH, flash_offs & (FLASH_PAGE_SIZE - 1));
    invalid_params_if(FLASH, count & (FLASH_PAGE_SIZE - 1));
    rom_connect_internal_flash_fn connect_internal_flash =
        (rom_connect_internal_flash_fn) rom_func_lookup_inline(ROM_FUNC_CONNECT_INTERNAL_FLASH);
    rom_flash_exit_xip_fn flash_exit_xip = (rom_flash_exit_xip_fn) rom_func_lookup_inline(ROM_FUNC_FLASH_EXIT_XIP);
    rom_flash_range_program_fn flash_range_program =
        (rom_flash_range_program_fn) rom_func_lookup_inline(ROM_FUNC_FLASH_RANGE_PROGRAM);
    assert(flash_exit_xip && connect_internal_flash && flash_range_program);

    __compiler_memory_barrier();

    connect_internal_flash();
    flash_exit_xip();
    flash_range_program(flash_offs, data, count);
}

//-----------------------------------------------------------------------------
// Lower-level flash access functions

// Bitbanging the chip select using IO overrides, in case RAM-resident IRQs
// are still running, and the FIFO bottoms out. (the bootrom does the same)
static void __no_inline_not_in_flash_func(flash_cs_force)(bool high) {
    uint32_t field_val =
        high ? IO_QSPI_GPIO_QSPI_SS_CTRL_OUTOVER_VALUE_HIGH : IO_QSPI_GPIO_QSPI_SS_CTRL_OUTOVER_VALUE_LOW;
    hw_write_masked(&ioqspi_hw->io[1].ctrl, field_val << IO_QSPI_GPIO_QSPI_SS_CTRL_OUTOVER_LSB,
                    IO_QSPI_GPIO_QSPI_SS_CTRL_OUTOVER_BITS);
}

void __no_inline_not_in_flash_func(flash_do_cmd)(const uint8_t *txbuf, uint8_t *rxbuf, size_t count) {
    rom_connect_internal_flash_fn connect_internal_flash =
        (rom_connect_internal_flash_fn) rom_func_lookup_inline(ROM_FUNC_CONNECT_INTERNAL_FLASH);
    rom_flash_exit_xip_fn flash_exit_xip = (rom_flash_exit_xip_fn) rom_func_lookup_inline(ROM_FUNC_FLASH_EXIT_XIP);
    assert(flash_exit_xip && connect_internal_flash);
    __compiler_memory_barrier();
    connect_internal_flash();
    flash_exit_xip();

    flash_cs_force(0);
    size_t tx_remaining = count;
    size_t rx_remaining = count;
    // We may be interrupted -- don't want FIFO to overflow if we're distracted.
    const size_t max_in_flight = 16 - 2;
    while (tx_remaining || rx_remaining) {
        uint32_t flags = ssi_hw->sr;
        bool can_put = !!(flags & SSI_SR_TFNF_BITS);
        bool can_get = !!(flags & SSI_SR_RFNE_BITS);
        if (can_put && tx_remaining && rx_remaining - tx_remaining < max_in_flight) {
            ssi_hw->dr0 = *txbuf++;
            --tx_remaining;
        }
        if (can_get && rx_remaining) {
            *rxbuf++ = (uint8_t) ssi_hw->dr0;
            --rx_remaining;
        }
    }
    flash_cs_force(1);
}

void flash_read(uint32_t flash_offs, uint8_t *data_out, size_t count) {
#ifdef PICO_FLASH_SIZE_BYTES
    hard_assert(flash_offs + count <= PICO_FLASH_SIZE_BYTES);
#endif
    invalid_params_if(FLASH, flash_offs & (FLASH_PAGE_SIZE - 1));
    invalid_params_if(FLASH, count & (FLASH_PAGE_SIZE - 1));
    rom_connect_internal_flash_fn connect_internal_flash =
        (rom_connect_internal_flash_fn) rom_func_lookup_inline(ROM_FUNC_CONNECT_INTERNAL_FLASH);
    rom_flash_exit_xip_fn flash_exit_xip = (rom_flash_exit_xip_fn) rom_func_lookup_inline(ROM_FUNC_FLASH_EXIT_XIP);
    assert(flash_exit_xip && connect_internal_flash);

    __compiler_memory_barrier();

    connect_internal_flash();
    flash_exit_xip();

    // As we exited XIP, we should be running in a compatible mode with a slow clock
    // 03h reads should be fine, as we shouldn't be beaming the clock

    uint8_t cmd_buf[] = { 0x03, (flash_offs >> 16) & 0xFF, (flash_offs >> 8) & 0xFF, (flash_offs) &0xFF };

    // Send command, starting with the command (discarding the command results), then receiving the actual data
    flash_cs_force(0);
    size_t tx_remaining = count + sizeof(cmd_buf);
    size_t rx_remaining = count + sizeof(cmd_buf);
    size_t cmd_tx_count = 0;
    size_t cmd_rx_count = 0;
    // We may be interrupted -- don't want FIFO to overflow if we're distracted.
    const size_t max_in_flight = 16 - 2;
    while (tx_remaining || rx_remaining) {
        uint32_t flags = ssi_hw->sr;
        bool can_put = !!(flags & SSI_SR_TFNF_BITS);
        bool can_get = !!(flags & SSI_SR_RFNE_BITS);
        if (can_put && tx_remaining && rx_remaining - tx_remaining < max_in_flight) {
            ssi_hw->dr0 = (cmd_tx_count < sizeof(cmd_buf) ? cmd_buf[cmd_tx_count++] : 0);
            --tx_remaining;
        }
        if (can_get && rx_remaining) {
            if (cmd_rx_count < sizeof(cmd_buf)) {
                (void) ssi_hw->dr0;
                cmd_rx_count++;
            }
            else {
                *data_out++ = (uint8_t) ssi_hw->dr0;
            }
            --rx_remaining;
        }
    }
    flash_cs_force(1);
}

// Use standard RUID command to get a unique identifier for the flash (and
// hence the board)

static_assert(FLASH_UNIQUE_ID_SIZE_BYTES == FLASH_RUID_DATA_BYTES, "");

void flash_get_unique_id(uint8_t *id_out) {
#if PICO_NO_FLASH
    __unused uint8_t *ignore = id_out;
    panic_unsupported();
#else
    uint8_t txbuf[FLASH_RUID_TOTAL_BYTES] = { 0 };
    uint8_t rxbuf[FLASH_RUID_TOTAL_BYTES] = { 0 };
    txbuf[0] = FLASH_RUID_CMD;
    flash_do_cmd(txbuf, rxbuf, FLASH_RUID_TOTAL_BYTES);
    for (int i = 0; i < FLASH_RUID_DATA_BYTES; i++)
        id_out[i] = rxbuf[i + 1 + FLASH_RUID_DUMMY_BYTES];
#endif
}
