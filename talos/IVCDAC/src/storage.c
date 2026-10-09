
#include "hardware/flash.h"
#include "hardware/sync.h"
#include "pico/stdlib.h"

#include <stdint.h>
#include <string.h>

#define STORAGE_SIZE (512 * 1024)  // 512kb
// should not encroach into program territory
#define STORAGE_OFFSET (PICO_FLASH_SIZE_BYTES - STORAGE_SIZE)  // top 512kb of flash
#define STORAGE_SECTOR_SIZE 4096                               // need to track which sectors were in

static const uint8_t *flash_target = (const uint8_t *) (XIP_BASE + STORAGE_OFFSET);
static uint8_t flash_page_buffer[FLASH_PAGE_SIZE];
static size_t page_buffer_pos = 0;
static uint32_t write_offset = 0;

static void check_flash_safe() {
    extern char __flash_binary_end;  // emitted from linker
    uint32_t program_end = (uint32_t) &__flash_binary_end - XIP_BASE;
    assert(STORAGE_OFFSET >= program_end);
}

void storage_write(uint8_t *data, size_t len) {
    // assert(write_offset + len <= STORAGE_SIZE);
    if (write_offset + len > STORAGE_SIZE) {
        return;
    }
    // else if (write_offset % STORAGE_SECTOR_SIZE == 0) {
    // }

    while (len > 0) {
        size_t remaining_page_space = FLASH_PAGE_SIZE - page_buffer_pos;
        size_t num_bytes_to_copy = len < remaining_page_space ? len : remaining_page_space;

        memcpy(flash_page_buffer + page_buffer_pos, data, num_bytes_to_copy);
        page_buffer_pos += num_bytes_to_copy;
        data += num_bytes_to_copy;
        len -= num_bytes_to_copy;

        if (page_buffer_pos == FLASH_PAGE_SIZE) {
            uint32_t interrupts = save_and_disable_interrupts();
            flash_range_program(STORAGE_OFFSET + write_offset, flash_page_buffer, FLASH_PAGE_SIZE);
            restore_interrupts(interrupts);

            write_offset += FLASH_PAGE_SIZE;
            page_buffer_pos = 0;
            memset(flash_page_buffer, 0xFF, FLASH_PAGE_SIZE);
        }
    }
}

bool storage_done_writing() {
    return write_offset >= STORAGE_SIZE;
}

void storage_flush() {
    if (page_buffer_pos == 0) {
        return;
    }

    uint32_t interrupts = save_and_disable_interrupts();
    flash_range_program(STORAGE_OFFSET + write_offset, flash_page_buffer, FLASH_PAGE_SIZE);
    restore_interrupts(interrupts);

    write_offset += FLASH_PAGE_SIZE;
    page_buffer_pos = 0;
    memset(flash_page_buffer, 0xFF, FLASH_PAGE_SIZE);
}

void storage_read(uint8_t *dest, size_t len) {
    assert(len <= STORAGE_SIZE);
    memcpy(dest, flash_target, len);
}

uint8_t storage_read_byte_at(uint32_t offset) {
    return (uint8_t) *(flash_target + offset);
}

void storage_init(bool will_write) {
    if (will_write) {
        check_flash_safe();

        memset(flash_page_buffer, 0xFF, FLASH_PAGE_SIZE);

        uint32_t interrupts = save_and_disable_interrupts();
        flash_range_erase(STORAGE_OFFSET, STORAGE_SIZE);
        restore_interrupts(interrupts);
    }
}
