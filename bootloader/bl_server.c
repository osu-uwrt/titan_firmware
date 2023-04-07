#include <assert.h>
#include <string.h>
#include "hardware/regs/addressmap.h"
#include "hardware/flash.h"

#include "canmore_titan/protocol.h"
#include "canmore_titan/reg_mapped_server.h"
#include "canmore_titan/bootloader_interface.h"

#include "build_version.h"

#include "crc32.h"
#include "bl_interface.h"
#include "bl_server.h"

// ========================================
// MCU Control Variables
// ========================================

static bool mcu_control_should_reboot = false;
static uint32_t mcu_control_magic_value = CANMORE_BL_MCU_CONTROL_MAGIC_VALUE;
static uint32_t mcu_control_release_type;

static union {
    uint8_t id_byte[FLASH_UNIQUE_ID_SIZE_BYTES];
    uint32_t id_word[2];
} mcu_control_flash_id;
static_assert(sizeof(mcu_control_flash_id.id_byte) == sizeof(mcu_control_flash_id.id_word), "Flash unique ID length does not match expected");
static_assert(sizeof(mcu_control_flash_id.id_byte) == FLASH_UNIQUE_ID_SIZE_BYTES, "Flash unique ID does not match expected size");

static bool reboot_mcu_cb(__unused const struct reg_mapped_server_register_definition *reg, __unused bool is_write, __unused uint32_t *data_ptr) {
    mcu_control_should_reboot = true;
    return true;
}

// ========================================
// Flash Control Variables
// ========================================

// Sanity check canmore parameters so that it matches with flash
static_assert(CANMORE_BL_FLASH_BUFFER_SIZE == FLASH_PAGE_SIZE, "Allocated flash buffer size does not match page size");
static_assert(CANMORE_BL_FLASH_READ_ADDR_ALIGN_MASK + 1 == FLASH_PAGE_SIZE, "Read addr alignment does not match page size");
static_assert(CANMORE_BL_FLASH_WRITE_ADDR_ALIGN_MASK + 1 == FLASH_PAGE_SIZE, "Write addr alignment does not match page size");
static_assert(CANMORE_BL_FLASH_ERASE_SIZE == FLASH_SECTOR_SIZE, "Erase size does not match sector size");
static_assert(CANMORE_BL_FLASH_ERASE_ADDR_ALIGN_MASK + 1 == FLASH_SECTOR_SIZE, "Erase addr alignment does not match sector size");

extern char __flash_app;

uint8_t flash_buffer[CANMORE_BL_FLASH_BUFFER_SIZE];
uint32_t flash_size = PICO_FLASH_SIZE_BYTES;
uint32_t flash_app_base = (uintptr_t)&__flash_app;
uint32_t flash_control_target_addr = 0;
uint32_t flash_control_bl_write_key = 0;
uint32_t flash_control_crc = 0;

static bool flash_control_command_cb(__unused const struct reg_mapped_server_register_definition *reg, bool is_write, uint32_t *data_ptr) {
    if (!is_write) return false;

    uint32_t command = *data_ptr;
    bool allow_bl_write = (flash_control_bl_write_key == CANMORE_BL_FLASH_CONTROL_BL_WRITE_KEY_VALUE);
    uintptr_t min_write_addr = (allow_bl_write ? XIP_MAIN_BASE : (uintptr_t)&__flash_app);
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
    DEFINE_REG_MEMORY_PTR(CANMORE_BL_MCU_CONTROL_MAJOR_VERSION_OFFSET, (uint32_t*)(&MAJOR_VERSION), REGISTER_PERM_READ_ONLY),
    DEFINE_REG_MEMORY_PTR(CANMORE_BL_MCU_CONTROL_MINOR_VERSION_OFFSET, (uint32_t*)(&MINOR_VERSION), REGISTER_PERM_READ_ONLY),
    DEFINE_REG_MEMORY_PTR(CANMORE_BL_MCU_CONTROL_RELEASE_TYPE_OFFSET, (uint32_t*)(&mcu_control_release_type), REGISTER_PERM_READ_ONLY),
    DEFINE_REG_MEMORY_PTR(CANMORE_BL_MCU_CONTROL_LOWER_FLASH_ID, &mcu_control_flash_id.id_word[0], REGISTER_PERM_READ_ONLY),
    DEFINE_REG_MEMORY_PTR(CANMORE_BL_MCU_CONTROL_UPPER_FLASH_ID, &mcu_control_flash_id.id_word[1], REGISTER_PERM_READ_ONLY),
    DEFINE_REG_EXEC_CALLBACK(CANMORE_BL_MCU_CONTROL_REBOOT_MCU_OFFSET, reboot_mcu_cb, REGISTER_PERM_WRITE_ONLY),
};

static reg_mapped_server_register_def_t bl_server_flash_control_regs[] = {
    DEFINE_REG_EXEC_CALLBACK(CANMORE_BL_FLASH_CONTROL_COMMAND_OFFSET, flash_control_command_cb, REGISTER_PERM_WRITE_ONLY),
    DEFINE_REG_MEMORY_PTR(CANMORE_BL_FLASH_CONTROL_TARGET_ADDR_OFFSET, &flash_control_target_addr, REGISTER_PERM_READ_WRITE),
    DEFINE_REG_MEMORY_PTR(CANMORE_BL_FLASH_CONTROL_BL_WRITE_KEY_OFFSET, &flash_control_bl_write_key, REGISTER_PERM_READ_WRITE),
    DEFINE_REG_MEMORY_PTR(CANMORE_BL_FLASH_CONTROL_FLASH_SIZE_OFFSET, &flash_size, REGISTER_PERM_READ_ONLY),
    DEFINE_REG_MEMORY_PTR(CANMORE_BL_FLASH_CONTROL_CRC_OFFSET, &flash_control_crc, REGISTER_PERM_READ_ONLY),
    DEFINE_REG_MEMORY_PTR(CANMORE_BL_FLASH_CONTROL_APP_BASE_OFFSET, &flash_app_base, REGISTER_PERM_READ_ONLY),
};

static reg_mapped_server_page_def_t bl_server_pages[] = {
    // General Control Region
    DEFINE_PAGE_REG_MAPPED(CANMORE_BL_MCU_CONTROL_PAGE_NUM, bl_server_mcu_control_regs),
    DEFINE_PAGE_UNIMPLEMENTED(CANMORE_BL_VERSION_STRING_PAGE_NUM),  // To be filled in by agent

    // Flash Control Region
    DEFINE_PAGE_REG_MAPPED(CANMORE_BL_FLASH_CONTROL_PAGE_NUM, bl_server_flash_control_regs),
    DEFINE_PAGE_MEMMAPPED_BYTE_ARRAY(CANMORE_BL_FLASH_BUFFER_PAGE_NUM, flash_buffer, REGISTER_PERM_READ_WRITE),
};

static reg_mapped_server_inst_t bl_server_inst = {
    .tx_func = &bl_interface_transmit,
    .page_array = bl_server_pages,
    .num_pages = sizeof(bl_server_pages)/sizeof(*bl_server_pages),
    .control_interface_mode = CANMORE_TITAN_CONTROL_INTERFACE_MODE_BOOTLOADER,
};

// ========================================
// Exported Functions
// ========================================

static uint8_t msg_buffer[REG_MAPPED_MAX_REQUEST_SIZE];

void bl_server_init(void) {
    // Fill out version
    mcu_control_release_type = RELEASE_TYPE;
    bl_server_pages[CANMORE_BL_VERSION_STRING_PAGE_NUM].page_type = PAGE_TYPE_MEMORY_MAPPED_BYTE;
    bl_server_pages[CANMORE_BL_VERSION_STRING_PAGE_NUM].type.mem_mapped_byte.perm = REGISTER_PERM_READ_ONLY;
    bl_server_pages[CANMORE_BL_VERSION_STRING_PAGE_NUM].type.mem_mapped_byte.base_addr = (uint8_t*) FULL_BUILD_TAG;
    bl_server_pages[CANMORE_BL_VERSION_STRING_PAGE_NUM].type.mem_mapped_byte.size = strlen(FULL_BUILD_TAG) + 1;

    // Fill out version ID
    flash_get_unique_id(mcu_control_flash_id.id_byte);
}

bool bl_server_tick(void) {
    size_t len;
    if (bl_interface_try_receive(msg_buffer, &len)) {
        reg_mapped_server_handle_request(&bl_server_inst, msg_buffer, len);
        return true;
    } else {
        return false;
    }
}

bool bl_server_check_for_magic_packet(void) {
    size_t len;
    if (bl_interface_try_receive(msg_buffer, &len)) {
        uint8_t boot_magic[] = CANMORE_TITAN_CONTROL_INTERFACE_BOOTLOADER_REQUEST;

        if (len != sizeof(boot_magic)) {
            return false;
        }

        return memcmp(boot_magic, msg_buffer, len) == 0;
    } else {
        return false;
    }
}

bool bl_server_should_reboot(void) {
    return mcu_control_should_reboot;
}