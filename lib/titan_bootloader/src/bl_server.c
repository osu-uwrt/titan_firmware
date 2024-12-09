#include "bl_server.h"

#include "bl_interface.h"

#include "canmore/crc32.h"
#include "canmore/protocol.h"
#include "canmore/reg_mapped/interface/bootloader.h"
#include "canmore/reg_mapped/server.h"
#include "hardware/flash.h"
#include "hardware/regs/addressmap.h"
#include "hardware/sync.h"
#include "titan/version.h"

#include <assert.h>
#include <string.h>

extern reg_mapped_server_inst_t bl_server_inst;  // Allows us to refer to the bl server later

// ========================================
// MCU Control Variables
// ========================================

static bool mcu_control_should_reboot = false;
static uint32_t mcu_control_magic_value = CANMORE_BL_MCU_CONTROL_MAGIC_VALUE;
static uint32_t mcu_control_major_version;
static uint32_t mcu_control_minor_version;
static uint32_t mcu_control_release_type;

static union {
    uint8_t id_byte[FLASH_UNIQUE_ID_SIZE_BYTES];
    uint32_t id_word[2];
} mcu_control_flash_id;
static_assert(sizeof(mcu_control_flash_id.id_byte) == sizeof(mcu_control_flash_id.id_word),
              "Flash unique ID length does not match expected");
static_assert(sizeof(mcu_control_flash_id.id_byte) == FLASH_UNIQUE_ID_SIZE_BYTES,
              "Flash unique ID does not match expected size");

static bool reboot_mcu_cb(__unused const struct reg_mapped_server_register_definition *reg, __unused bool is_write,
                          __unused uint32_t *data_ptr) {
    mcu_control_should_reboot = true;
    return true;
}

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
    bool is_in_flash = false;
    bool is_pc_read = false;
    bool is_sp_read = false;
    bool is_lr_read = false;

    // This function is extra convoluted since we want to capture the memory and PC/SP at the same location, so we get
    // accurate stack reconstruction. Yay ARM ABI!

    // Do register decoding
    if (bl_server_inst.num_pages < CANMORE_BL_GDB_STUB_PAGE_NUM ||
        bl_server_inst.page_array[CANMORE_BL_GDB_STUB_PAGE_NUM].page_type != PAGE_TYPE_REGISTER_MAPPED)
        return false;
    size_t reg_cnt = bl_server_inst.page_array[CANMORE_BL_GDB_STUB_PAGE_NUM].type.reg_mapped.num_registers;
    const reg_mapped_server_register_def_t *page_def =
        bl_server_inst.page_array[CANMORE_BL_GDB_STUB_PAGE_NUM].type.reg_mapped.reg_array;

    // This is a read word address
    if (reg_cnt > CANMORE_BL_GDB_STUB_READ_WORD_ADDR_OFFSET &&
        reg == &page_def[CANMORE_BL_GDB_STUB_READ_WORD_ADDR_OFFSET]) {
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
            addr = (addr & 0x00FFFFFF);
            is_in_flash = true;
        }
    }
    else if (reg_cnt > CANMORE_BL_GDB_STUB_PC_REGISTER_OFFSET &&
             reg == &page_def[CANMORE_BL_GDB_STUB_PC_REGISTER_OFFSET]) {
        is_pc_read = true;
    }
    else if (reg_cnt > CANMORE_BL_GDB_STUB_SP_REGISTER_OFFSET &&
             reg == &page_def[CANMORE_BL_GDB_STUB_SP_REGISTER_OFFSET]) {
        is_sp_read = true;
    }
    else if (reg_cnt > CANMORE_BL_GDB_STUB_LR_REGISTER_OFFSET &&
             reg == &page_def[CANMORE_BL_GDB_STUB_LR_REGISTER_OFFSET]) {
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
    register uint32_t val = 0;
    if (is_pc_read)
        pico_default_asm_volatile("mov %0, pc\n" : "=r"(val));
    else if (is_sp_read)
        pico_default_asm_volatile("mov %0, sp\n" : "=r"(val));
    else if (is_lr_read)
        pico_default_asm_volatile("mov %0, lr\n" : "=r"(val));
    else if (!is_in_flash)
        pico_default_asm_volatile("ldr %0, [%1]" : "=r"(val) : "r"(addr));
    __dsb();
    __isb();

    if (is_sp_read || is_pc_read || is_lr_read) {
        if (is_write) {
            return false;  // PC, SP, and LR reads must be reads
        }
        *data_ptr = val;
    }
    else if (is_in_flash) {
        uint32_t data;
        flash_read(addr, (uint8_t *) &data, sizeof(data));
        gdb_stub_memory_data = data;
    }
    else {
        // It's a memory fetch, store into the appropriate register
        gdb_stub_memory_data = val;
    }
    return true;
}

// ========================================
// Flash Control Variables
// ========================================

// Sanity check canmore parameters so that it matches with flash
static_assert(CANMORE_BL_FLASH_BUFFER_SIZE == FLASH_PAGE_SIZE, "Allocated flash buffer size does not match page size");
static_assert(CANMORE_BL_FLASH_READ_ADDR_ALIGN_MASK + 1 == FLASH_PAGE_SIZE,
              "Read addr alignment does not match page size");
static_assert(CANMORE_BL_FLASH_WRITE_ADDR_ALIGN_MASK + 1 == FLASH_PAGE_SIZE,
              "Write addr alignment does not match page size");
static_assert(CANMORE_BL_FLASH_ERASE_SIZE == FLASH_SECTOR_SIZE, "Erase size does not match sector size");
static_assert(CANMORE_BL_FLASH_ERASE_ADDR_ALIGN_MASK + 1 == FLASH_SECTOR_SIZE,
              "Erase addr alignment does not match sector size");

extern char __flash_app;

uint8_t flash_buffer[CANMORE_BL_FLASH_BUFFER_SIZE];
uint32_t flash_size = PICO_FLASH_SIZE_BYTES;
uint32_t flash_app_base = (uintptr_t) &__flash_app;
uint32_t flash_control_target_addr = 0;
uint32_t flash_control_bl_write_key = 0;
uint32_t flash_control_crc = 0;

static bool flash_control_command_cb(__unused const struct reg_mapped_server_register_definition *reg, bool is_write,
                                     uint32_t *data_ptr) {
    if (!is_write)
        return false;

    uint32_t command = *data_ptr;
    bool allow_bl_write = (flash_control_bl_write_key == CANMORE_BL_FLASH_CONTROL_BL_WRITE_KEY_VALUE);
    uintptr_t min_write_addr = (allow_bl_write ? XIP_MAIN_BASE : (uintptr_t) &__flash_app);
    uintptr_t min_read_addr = XIP_MAIN_BASE;
    uintptr_t max_page_addr = (XIP_MAIN_BASE + PICO_FLASH_SIZE_BYTES - CANMORE_BL_FLASH_BUFFER_SIZE);
    uintptr_t max_sector_addr = (XIP_MAIN_BASE + PICO_FLASH_SIZE_BYTES - CANMORE_BL_FLASH_ERASE_SIZE);

    if (command == CANMORE_BL_FLASH_CONTROL_COMMAND_READ) {
        if (flash_control_target_addr < min_read_addr || flash_control_target_addr > max_page_addr) {
            return false;
        }
        if ((flash_control_target_addr & CANMORE_BL_FLASH_READ_ADDR_ALIGN_MASK) != 0) {
            return false;
        }

        flash_read(flash_control_target_addr - XIP_MAIN_BASE, flash_buffer, CANMORE_BL_FLASH_BUFFER_SIZE);

        return true;
    }
    else if (command == CANMORE_BL_FLASH_CONTROL_COMMAND_WRITE) {
        if (flash_control_target_addr < min_write_addr || flash_control_target_addr > max_page_addr) {
            return false;
        }
        if ((flash_control_target_addr & CANMORE_BL_FLASH_WRITE_ADDR_ALIGN_MASK) != 0) {
            return false;
        }

        flash_range_program(flash_control_target_addr - XIP_MAIN_BASE, flash_buffer, CANMORE_BL_FLASH_BUFFER_SIZE);
        return true;
    }
    else if (command == CANMORE_BL_FLASH_CONTROL_COMMAND_ERASE) {
        if (flash_control_target_addr < min_write_addr || flash_control_target_addr > max_sector_addr) {
            return false;
        }
        if ((flash_control_target_addr & CANMORE_BL_FLASH_ERASE_ADDR_ALIGN_MASK) != 0) {
            return false;
        }

        flash_range_erase(flash_control_target_addr - XIP_MAIN_BASE, CANMORE_BL_FLASH_ERASE_SIZE);
        return true;
    }
    else if (command == CANMORE_BL_FLASH_CONTROL_COMMAND_CRC) {
        // Compute crc32 of flash_buffer
        flash_control_crc = crc32_compute(flash_buffer, sizeof(flash_buffer));
        return true;
    }
    else {
        return false;
    }
}

// ========================================
// Register Map
// ========================================

static reg_mapped_server_register_def_t bl_server_mcu_control_regs[] = {
    DEFINE_REG_MEMORY_PTR(CANMORE_BL_MCU_CONTROL_MAGIC_OFFSET, &mcu_control_magic_value, REGISTER_PERM_READ_ONLY),
    DEFINE_REG_MEMORY_PTR(CANMORE_BL_MCU_CONTROL_MAJOR_VERSION_OFFSET, &mcu_control_major_version,
                          REGISTER_PERM_READ_ONLY),
    DEFINE_REG_MEMORY_PTR(CANMORE_BL_MCU_CONTROL_MINOR_VERSION_OFFSET, &mcu_control_minor_version,
                          REGISTER_PERM_READ_ONLY),
    DEFINE_REG_MEMORY_PTR(CANMORE_BL_MCU_CONTROL_RELEASE_TYPE_OFFSET, &mcu_control_release_type,
                          REGISTER_PERM_READ_ONLY),
    DEFINE_REG_MEMORY_PTR(CANMORE_BL_MCU_CONTROL_LOWER_FLASH_ID, &mcu_control_flash_id.id_word[0],
                          REGISTER_PERM_READ_ONLY),
    DEFINE_REG_MEMORY_PTR(CANMORE_BL_MCU_CONTROL_UPPER_FLASH_ID, &mcu_control_flash_id.id_word[1],
                          REGISTER_PERM_READ_ONLY),
    DEFINE_REG_EXEC_CALLBACK(CANMORE_BL_MCU_CONTROL_REBOOT_MCU_OFFSET, reboot_mcu_cb, REGISTER_PERM_WRITE_ONLY),
};

static const reg_mapped_server_register_def_t bl_server_gdb_stub_regs[] = {
    DEFINE_REG_EXEC_CALLBACK(CANMORE_BL_GDB_STUB_READ_WORD_ADDR_OFFSET, gdb_stub_read_mem_cb, REGISTER_PERM_WRITE_ONLY),
    DEFINE_REG_EXEC_CALLBACK(CANMORE_BL_GDB_STUB_WRITE_WORD_ADDR_OFFSET, gdb_stub_write_mem_cb,
                             REGISTER_PERM_WRITE_ONLY),
    DEFINE_REG_MEMORY_PTR(CANMORE_BL_GDB_STUB_MEMORY_DATA_OFFSET, &gdb_stub_memory_data, REGISTER_PERM_READ_WRITE),
    DEFINE_REG_EXEC_CALLBACK(CANMORE_BL_GDB_STUB_PC_REGISTER_OFFSET, gdb_stub_read_mem_cb, REGISTER_PERM_READ_ONLY),
    DEFINE_REG_EXEC_CALLBACK(CANMORE_BL_GDB_STUB_SP_REGISTER_OFFSET, gdb_stub_read_mem_cb, REGISTER_PERM_READ_ONLY),
    DEFINE_REG_EXEC_CALLBACK(CANMORE_BL_GDB_STUB_LR_REGISTER_OFFSET, gdb_stub_read_mem_cb, REGISTER_PERM_READ_ONLY),
};

static reg_mapped_server_register_def_t bl_server_flash_control_regs[] = {
    DEFINE_REG_EXEC_CALLBACK(CANMORE_BL_FLASH_CONTROL_COMMAND_OFFSET, flash_control_command_cb,
                             REGISTER_PERM_WRITE_ONLY),
    DEFINE_REG_MEMORY_PTR(CANMORE_BL_FLASH_CONTROL_TARGET_ADDR_OFFSET, &flash_control_target_addr,
                          REGISTER_PERM_READ_WRITE),
    DEFINE_REG_MEMORY_PTR(CANMORE_BL_FLASH_CONTROL_BL_WRITE_KEY_OFFSET, &flash_control_bl_write_key,
                          REGISTER_PERM_READ_WRITE),
    DEFINE_REG_MEMORY_PTR(CANMORE_BL_FLASH_CONTROL_FLASH_SIZE_OFFSET, &flash_size, REGISTER_PERM_READ_ONLY),
    DEFINE_REG_MEMORY_PTR(CANMORE_BL_FLASH_CONTROL_CRC_OFFSET, &flash_control_crc, REGISTER_PERM_READ_ONLY),
    DEFINE_REG_MEMORY_PTR(CANMORE_BL_FLASH_CONTROL_APP_BASE_OFFSET, &flash_app_base, REGISTER_PERM_READ_ONLY),
};

static reg_mapped_server_page_def_t bl_server_pages[] = {
    // General Control Region
    DEFINE_PAGE_REG_MAPPED(CANMORE_BL_MCU_CONTROL_PAGE_NUM, bl_server_mcu_control_regs),
    DEFINE_PAGE_UNIMPLEMENTED(CANMORE_BL_VERSION_STRING_PAGE_NUM),  // To be filled in at init
    DEFINE_PAGE_REG_MAPPED(CANMORE_BL_GDB_STUB_PAGE_NUM, bl_server_gdb_stub_regs),

    // Flash Control Region
    DEFINE_PAGE_REG_MAPPED(CANMORE_BL_FLASH_CONTROL_PAGE_NUM, bl_server_flash_control_regs),
    DEFINE_PAGE_MEMMAPPED_BYTE_ARRAY(CANMORE_BL_FLASH_BUFFER_PAGE_NUM, flash_buffer, REGISTER_PERM_READ_WRITE),
};

#if !CANMORE_CONFIG_DISABLE_MULTIWORD
// Buffer for multiword responses
static uint8_t multiword_resp_buffer[BL_INTERFACE_MAX_PACKET_LEN];
#endif

reg_mapped_server_inst_t bl_server_inst = {
    .tx_func = &bl_interface_transmit,
    .page_array = bl_server_pages,
    .num_pages = sizeof(bl_server_pages) / sizeof(*bl_server_pages),
    .control_interface_mode = CANMORE_CONTROL_INTERFACE_MODE_BOOTLOADER,

#if !CANMORE_CONFIG_DISABLE_MULTIWORD
    // If we have multiword, provide our allocated buffer above
    .multiword_resp_buffer = (reg_mapped_response_t *) multiword_resp_buffer,
    .multiword_resp_buffer_max_count = REG_MAPPED_COMPUTE_MAX_RESP_WORD_COUNT(BL_INTERFACE_MAX_PACKET_LEN),
#endif
};

// ========================================
// Exported Functions
// ========================================

static uint8_t msg_buffer[BL_INTERFACE_MAX_PACKET_LEN];

void bl_server_init(void) {
    // Fill out version
    mcu_control_major_version = MAJOR_VERSION;
    mcu_control_minor_version = MINOR_VERSION;
    mcu_control_release_type = RELEASE_TYPE;
    bl_server_pages[CANMORE_BL_VERSION_STRING_PAGE_NUM].page_type = PAGE_TYPE_MEMORY_MAPPED_BYTE;
    bl_server_pages[CANMORE_BL_VERSION_STRING_PAGE_NUM].type.mem_mapped_byte.perm = REGISTER_PERM_READ_ONLY;
    bl_server_pages[CANMORE_BL_VERSION_STRING_PAGE_NUM].type.mem_mapped_byte.base_addr = (uint8_t *) FULL_BUILD_TAG;
    bl_server_pages[CANMORE_BL_VERSION_STRING_PAGE_NUM].type.mem_mapped_byte.size = strlen(FULL_BUILD_TAG) + 1;

    // Fill out board ID
    flash_get_unique_id(mcu_control_flash_id.id_byte);
}

bool bl_server_tick(void) {
    size_t len;
    if (bl_interface_try_receive(msg_buffer, &len)) {
        reg_mapped_server_handle_request(&bl_server_inst, msg_buffer, len);
        return true;
    }
    else {
        return false;
    }
}

bool bl_server_check_for_magic_packet(void) {
    size_t len;
    if (bl_interface_try_receive(msg_buffer, &len)) {
        uint8_t boot_magic[] = CANMORE_CONTROL_INTERFACE_BOOTLOADER_REQUEST;

        if (len != sizeof(boot_magic)) {
            return false;
        }

        return memcmp(boot_magic, msg_buffer, len) == 0;
    }
    else {
        return false;
    }
}

bool bl_server_should_reboot(void) {
    return mcu_control_should_reboot;
}
