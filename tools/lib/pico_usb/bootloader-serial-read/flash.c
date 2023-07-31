/*
 * Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

//#include "pico/bootrom.h"

#include <stdint.h>
#include <stdbool.h>
#include "hardware/regs/addressmap.h"
#include "hardware/regs/ssi.h"
#include "hardware/regs/io_qspi.h"

// Defined in linker script
extern void* serial_write_address;

// Bootrom Definitions

typedef uint32_t size_t;

#define ROM_FUNC_POPCOUNT32             ROM_TABLE_CODE('P', '3')
#define ROM_FUNC_REVERSE32              ROM_TABLE_CODE('R', '3')
#define ROM_FUNC_CLZ32                  ROM_TABLE_CODE('L', '3')
#define ROM_FUNC_CTZ32                  ROM_TABLE_CODE('T', '3')
#define ROM_FUNC_MEMSET                 ROM_TABLE_CODE('M', 'S')
#define ROM_FUNC_MEMSET4                ROM_TABLE_CODE('S', '4')
#define ROM_FUNC_MEMCPY                 ROM_TABLE_CODE('M', 'C')
#define ROM_FUNC_MEMCPY44               ROM_TABLE_CODE('C', '4')
#define ROM_FUNC_RESET_USB_BOOT         ROM_TABLE_CODE('U', 'B')
#define ROM_FUNC_CONNECT_INTERNAL_FLASH ROM_TABLE_CODE('I', 'F')
#define ROM_FUNC_FLASH_EXIT_XIP         ROM_TABLE_CODE('E', 'X')
#define ROM_FUNC_FLASH_RANGE_ERASE      ROM_TABLE_CODE('R', 'E')
#define ROM_FUNC_FLASH_RANGE_PROGRAM    ROM_TABLE_CODE('R', 'P')
#define ROM_FUNC_FLASH_FLUSH_CACHE      ROM_TABLE_CODE('F', 'C')
#define ROM_FUNC_FLASH_ENTER_CMD_XIP    ROM_TABLE_CODE('C', 'X')
#define ROM_TABLE_CODE(c1, c2) ((c1) | ((c2) << 8))
typedef uint32_t (*rom_popcount32_fn)(uint32_t);
typedef uint32_t (*rom_reverse32_fn)(uint32_t);
typedef uint32_t (*rom_clz32_fn)(uint32_t);
typedef uint32_t (*rom_ctz32_fn)(uint32_t);
typedef uint8_t *(*rom_memset_fn)(uint8_t *, uint8_t, uint32_t);
typedef uint32_t *(*rom_memset4_fn)(uint32_t *, uint8_t, uint32_t);
typedef uint32_t *(*rom_memcpy_fn)(uint8_t *, const uint8_t *, uint32_t);
typedef uint32_t *(*rom_memcpy44_fn)(uint32_t *, const uint32_t *, uint32_t);
typedef void __attribute__((noreturn)) (*rom_reset_usb_boot_fn)(uint32_t, uint32_t);
typedef rom_reset_usb_boot_fn reset_usb_boot_fn; // kept for backwards compatibility
typedef void (*rom_connect_internal_flash_fn)(void);
typedef void (*rom_flash_exit_xip_fn)(void);
typedef void (*rom_flash_range_erase_fn)(uint32_t, size_t, uint32_t, uint8_t);
typedef void (*rom_flash_range_program_fn)(uint32_t, const uint8_t*, size_t);
typedef void (*rom_flash_flush_cache_fn)(void);
typedef void (*rom_flash_enter_cmd_xip_fn)(void);
typedef void *(*rom_table_lookup_fn)(uint16_t *table, uint32_t code);

#define rom_hword_as_ptr(rom_address) (void *)(uintptr_t)(*(uint16_t *)rom_address)
__attribute__((always_inline)) static inline void *rom_func_lookup_inline(uint32_t code) {
    rom_table_lookup_fn rom_table_lookup = (rom_table_lookup_fn) rom_hword_as_ptr(0x18);
    uint16_t *func_table = (uint16_t *) rom_hword_as_ptr(0x14);
    return rom_table_lookup(func_table, code);
}

// Standard RUID instruction: 4Bh command prefix, 32 dummy bits, 64 data bits.
#define FLASH_RUID_CMD 0x4b
#define FLASH_RUID_DUMMY_BYTES 4
#define FLASH_RUID_DATA_BYTES 8
#define FLASH_RUID_TOTAL_BYTES (1 + FLASH_RUID_DUMMY_BYTES + FLASH_RUID_DATA_BYTES)

void * memset ( void * ptr, int value, size_t num ) {
    rom_memset_fn rom_memset = (rom_memset_fn)rom_func_lookup_inline(ROM_FUNC_MEMSET);
    return (rom_memset)(ptr, value, num);
}

//-----------------------------------------------------------------------------
// Lower-level flash access functions

// Bitbanging the chip select using IO overrides, in case RAM-resident IRQs
// are still running, and the FIFO bottoms out. (the bootrom does the same)
static void flash_cs_force(bool high) {
    volatile uint32_t* ss_ctrl_io = (volatile uint32_t*)(IO_QSPI_BASE + IO_QSPI_GPIO_QSPI_SS_CTRL_OFFSET);
    uint32_t field_val = high ?
        IO_QSPI_GPIO_QSPI_SS_CTRL_OUTOVER_VALUE_HIGH :
        IO_QSPI_GPIO_QSPI_SS_CTRL_OUTOVER_VALUE_LOW;
    *ss_ctrl_io = ((*ss_ctrl_io) & ~(IO_QSPI_GPIO_QSPI_SS_CTRL_OUTOVER_BITS)) | (field_val << IO_QSPI_GPIO_QSPI_SS_CTRL_OUTOVER_LSB);
}

void flash_do_cmd(const uint8_t *txbuf, uint8_t *rxbuf, size_t count) {
    volatile uint32_t* ssi_hw_sr_io = (volatile uint32_t*)(XIP_SSI_BASE + SSI_SR_OFFSET);
    volatile uint32_t* ssi_hw_dr0_io = (volatile uint32_t*)(XIP_SSI_BASE + SSI_DR0_OFFSET);
    rom_connect_internal_flash_fn connect_internal_flash = (rom_connect_internal_flash_fn)rom_func_lookup_inline(ROM_FUNC_CONNECT_INTERNAL_FLASH);
    rom_flash_flush_cache_fn flash_flush_cache = (rom_flash_flush_cache_fn)rom_func_lookup_inline(ROM_FUNC_FLASH_FLUSH_CACHE);
    connect_internal_flash();

    flash_cs_force(0);
    size_t tx_remaining = count;
    size_t rx_remaining = count;
    // We may be interrupted -- don't want FIFO to overflow if we're distracted.
    const size_t max_in_flight = 16 - 2;
    while (tx_remaining || rx_remaining) {
        uint32_t flags = (*ssi_hw_sr_io);
        bool can_put = !!(flags & SSI_SR_TFNF_BITS);
        bool can_get = !!(flags & SSI_SR_RFNE_BITS);
        if (can_put && tx_remaining && rx_remaining - tx_remaining < max_in_flight) {
            *ssi_hw_dr0_io = *txbuf++;
            --tx_remaining;
        }
        if (can_get && rx_remaining) {
            *rxbuf++ = (uint8_t)(*ssi_hw_dr0_io);
            --rx_remaining;
        }
    }
    flash_cs_force(1);

    flash_flush_cache();
}

// Use standard RUID command to get a unique identifier for the flash (and
// hence the board)

void flash_get_unique_id(uint8_t *id_out) {
    uint8_t txbuf[FLASH_RUID_TOTAL_BYTES] = {0};
    uint8_t rxbuf[FLASH_RUID_TOTAL_BYTES] = {0};
    txbuf[0] = FLASH_RUID_CMD;
    flash_do_cmd(txbuf, rxbuf, FLASH_RUID_TOTAL_BYTES);
    for (int i = 0; i < FLASH_RUID_DATA_BYTES; i++)
        id_out[i] = rxbuf[i + 1 + FLASH_RUID_DUMMY_BYTES];
}

__attribute__ ((section (".text.entry"))) void entry(void) {
//    flash_get_unique_id((uint8_t*)(serial_write_address));
    flash_get_unique_id((uint8_t*)(0x20001000));
}

