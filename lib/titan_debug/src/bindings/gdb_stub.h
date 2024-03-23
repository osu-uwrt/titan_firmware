#include "../titan_debug_internal.h"

#include "hardware/sync.h"

// This file is designed to be included in the debug_server.c
// These headers each take a various page (or set of related pages) as part of the canmore debug protocol
// See canmore/debug_interface.h for more information about the underlying protocol implemented in these files

// ========================================
// GDB Stub Bindings
// ========================================

static uint32_t gdb_stub_memory_data = 0;

static bool gdb_stub_write_mem_cb(__unused const struct reg_mapped_server_register_definition *reg,
                                  __unused bool is_write, uint32_t *data_ptr) {
    uintptr_t addr = *data_ptr;
    if (addr % 4 != 0) {
        // Don't allow unaligned writes
        return false;
    }

    if (addr >= SRAM_BASE && addr < SRAM_END)  // Striped addresses (& scratch_x + scratch_y)
        ;
    else if (addr >= SRAM0_BASE && addr < 0x21040000)  // Non-striped RAM
        ;
    else if (addr >= 0x40000000 && addr < 0x40070000)  // APB Peripherals
        ;
    else if (addr >= 0x50000000 && addr < 0x50400000)  // AHB Peripherals
        ;
    else if (addr >= SIO_BASE && addr < SIO_BASE + 0x180)  // SIO Peripherals
        ;
    else if (addr >= PPB_BASE + 0x0000e000 && addr < PPB_BASE + 0x0000f000)  // ARM Cortex SCS
        ;
    else
        return false;

    // Perform the write
    uint32_t *mem_ptr = (uint32_t *) addr;
    *mem_ptr = gdb_stub_memory_data;
    return true;
}

static bool gdb_stub_read_mem_cb(const struct reg_mapped_server_register_definition *reg, bool is_write,
                                 uint32_t *data_ptr) {
    register uint32_t addr = 0;
    bool is_pc_read = false;
    bool is_sp_read = false;
    bool is_lr_read = false;

    // This function is extra convoluted since we want to capture the memory and PC/SP at the same location, so we get
    // accurate stack reconstruction. Yay ARM ABI!

    // Do register decoding
    extern reg_mapped_server_inst_t debug_server_inst;  // Need to pull in debug_server to do decoding on it

    if (debug_server_inst.num_pages < CANMORE_DBG_GDB_STUB_PAGE_NUM ||
        debug_server_inst.page_array[CANMORE_DBG_GDB_STUB_PAGE_NUM].page_type != PAGE_TYPE_REGISTER_MAPPED)
        return false;
    size_t reg_cnt = debug_server_inst.page_array[CANMORE_DBG_GDB_STUB_PAGE_NUM].type.reg_mapped.num_registers;
    const reg_mapped_server_register_def_t *page_def =
        debug_server_inst.page_array[CANMORE_DBG_GDB_STUB_PAGE_NUM].type.reg_mapped.reg_array;

    // This is a read word address
    if (reg_cnt > CANMORE_DBG_GDB_STUB_READ_WORD_ADDR_OFFSET &&
        reg == &page_def[CANMORE_DBG_GDB_STUB_READ_WORD_ADDR_OFFSET]) {
        // Read word will write the value to fetch, if it's a read, they're using it wrong
        if (!is_write) {
            return false;
        }
        addr = *data_ptr;

        if (addr % 4 != 0) {
            // Address must be word aligned
            return false;
        }

        // Check that address is in range
        static_assert(ROM_BASE == 0, "Assuming rom tsarts at 0 to silence compiler warning");

        if (addr >= SRAM_BASE && addr < SRAM_END)  // Striped addresses (& scratch_x + scratch_y)
            ;
        else if (addr >= XIP_MAIN_BASE && addr < XIP_MAIN_BASE + PICO_FLASH_SIZE_BYTES)  // Flash normal
            ;
        else if (addr >= XIP_NOALLOC_BASE && addr < XIP_NOALLOC_BASE + PICO_FLASH_SIZE_BYTES)  // Flash noalloc
            ;
        else if (addr >= XIP_NOCACHE_BASE && addr < XIP_NOCACHE_BASE + PICO_FLASH_SIZE_BYTES)  // Flash noalloc
            ;
        else if (addr >= XIP_NOCACHE_NOALLOC_BASE && addr < XIP_NOCACHE_NOALLOC_BASE + PICO_FLASH_SIZE_BYTES)
            ;
        else if (addr < (16 * 1024))  // ROM (Hardcoded from datasheet saying that ROM is 16K)
            ;
        else if (addr >= SRAM0_BASE && addr < 0x21040000)  // Non-striped RAM
            ;
        else if (addr >= 0x40000000 && addr < 0x40070000)  // APB Peripherals
            ;
        else if (addr >= 0x50000000 && addr < 0x50400000)  // AHB Peripherals
            ;
        else if (addr >= SIO_BASE && addr < SIO_BASE + 0x180)  // SIO Peripherals
            ;
        else if (addr >= PPB_BASE + 0x0000e000 && addr < PPB_BASE + 0x0000f000)  // ARM Cortex SCS
            ;
        else {
            // Address out of range
            return false;
        }

        // If address is in flash, only do noalloc reads so we don't muddy up cache
        if ((addr & 0xFC000000) == XIP_BASE) {
            addr = XIP_NOALLOC_BASE + (addr & 0x00FFFFFF);
        }
    }
    else if (reg_cnt > CANMORE_DBG_GDB_STUB_PC_REGISTER_OFFSET &&
             reg == &page_def[CANMORE_DBG_GDB_STUB_PC_REGISTER_OFFSET]) {
        is_pc_read = true;
    }
    else if (reg_cnt > CANMORE_DBG_GDB_STUB_SP_REGISTER_OFFSET &&
             reg == &page_def[CANMORE_DBG_GDB_STUB_SP_REGISTER_OFFSET]) {
        is_sp_read = true;
    }
    else if (reg_cnt > CANMORE_DBG_GDB_STUB_LR_REGISTER_OFFSET &&
             reg == &page_def[CANMORE_DBG_GDB_STUB_LR_REGISTER_OFFSET]) {
        is_lr_read = true;
    }
    else {
        // We don't know what it is
        return false;
    }

    // Perform the read. This must be protected so the compiler doesn't optimize it away
    // If these get optimized apart, then GDB will get a PC and SP that are misaligned, and you won't be able to see
    // local variables.
    __dsb();
    __isb();
    register uint32_t val;
    if (is_pc_read)
        pico_default_asm_volatile("mov %0, pc\n" : "=r"(val));
    else if (is_sp_read)
        pico_default_asm_volatile("mov %0, sp\n" : "=r"(val));
    else if (is_lr_read)
        pico_default_asm_volatile("mov %0, lr\n" : "=r"(val));
    else
        pico_default_asm_volatile("ldr %0, [%1]" : "=r"(val) : "r"(addr));
    __dsb();
    __isb();

    if (is_sp_read || is_pc_read || is_lr_read) {
        if (is_write) {
            return false;  // PC, SP, and LR reads must be reads
        }
        *data_ptr = val;
    }
    else {
        // It's a memory fetch, store into the appropriate register
        gdb_stub_memory_data = val;
    }
    return true;
}

// ========================================
// Exported Page
// ========================================

static const reg_mapped_server_register_def_t debug_server_gdb_stub_regs[] = {
    DEFINE_REG_EXEC_CALLBACK(CANMORE_DBG_GDB_STUB_READ_WORD_ADDR_OFFSET, gdb_stub_read_mem_cb,
                             REGISTER_PERM_WRITE_ONLY),
    DEFINE_REG_EXEC_CALLBACK(CANMORE_DBG_GDB_STUB_WRITE_WORD_ADDR_OFFSET, gdb_stub_write_mem_cb,
                             REGISTER_PERM_WRITE_ONLY),
    DEFINE_REG_MEMORY_PTR(CANMORE_DBG_GDB_STUB_MEMORY_DATA_OFFSET, &gdb_stub_memory_data, REGISTER_PERM_READ_WRITE),
    DEFINE_REG_EXEC_CALLBACK(CANMORE_DBG_GDB_STUB_PC_REGISTER_OFFSET, gdb_stub_read_mem_cb, REGISTER_PERM_READ_ONLY),
    DEFINE_REG_EXEC_CALLBACK(CANMORE_DBG_GDB_STUB_SP_REGISTER_OFFSET, gdb_stub_read_mem_cb, REGISTER_PERM_READ_ONLY),
    DEFINE_REG_EXEC_CALLBACK(CANMORE_DBG_GDB_STUB_LR_REGISTER_OFFSET, gdb_stub_read_mem_cb, REGISTER_PERM_READ_ONLY),
};
