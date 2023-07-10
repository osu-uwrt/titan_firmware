#include "pico/unique_id.h"

#include "titan/debug.h"

#if TITAN_SAFETY
#include "titan/safety.h"
#include "titan/version.h"
#include "hardware/watchdog.h"

// ========================================
// Bindings
// ========================================

static uint32_t mcu_control_magic_value = CANMORE_DBG_MCU_CONTROL_MAGIC_VALUE;
static uint32_t mcu_control_major_version;
static uint32_t mcu_control_minor_version;
static uint32_t mcu_control_release_type;
static bool enter_bootloader_on_return = false;
static bool reboot_mcu_on_return = false;

static union {
    pico_unique_board_id_t id_pico;
    uint32_t id_word[2];
} mcu_control_flash_id;
static_assert(sizeof(mcu_control_flash_id.id_word) == PICO_UNIQUE_BOARD_ID_SIZE_BYTES, "Flash unique ID does not match expected size");

static bool enter_bl_cb(__unused const struct reg_mapped_server_register_definition *reg, __unused bool is_write, __unused uint32_t *data_ptr) {
    enter_bootloader_on_return = true;
    return true;
}

static bool reboot_mcu_cb(__unused const struct reg_mapped_server_register_definition *reg, __unused bool is_write, __unused uint32_t *data_ptr) {
    reboot_mcu_on_return = true;
    return true;
}

// ========================================
// Register Map
// ========================================

static reg_mapped_server_register_def_t debug_server_mcu_control_regs[] = {
    DEFINE_REG_MEMORY_PTR(CANMORE_DBG_MCU_CONTROL_MAGIC_OFFSET, &mcu_control_magic_value, REGISTER_PERM_READ_ONLY),
    DEFINE_REG_EXEC_CALLBACK(CANMORE_DBG_MCU_CONTROL_ENTER_BL_OFFSET, enter_bl_cb, REGISTER_PERM_WRITE_ONLY),
    DEFINE_REG_MEMORY_PTR(CANMORE_DBG_MCU_CONTROL_LOWER_FLASH_ID, &mcu_control_flash_id.id_word[0], REGISTER_PERM_READ_ONLY),
    DEFINE_REG_MEMORY_PTR(CANMORE_DBG_MCU_CONTROL_UPPER_FLASH_ID, &mcu_control_flash_id.id_word[1], REGISTER_PERM_READ_ONLY),
    DEFINE_REG_MEMORY_PTR(CANMORE_DBG_MCU_CONTROL_MAJOR_VERSION_OFFSET, &mcu_control_major_version, REGISTER_PERM_READ_ONLY),
    DEFINE_REG_MEMORY_PTR(CANMORE_DBG_MCU_CONTROL_MINOR_VERSION_OFFSET, &mcu_control_minor_version, REGISTER_PERM_READ_ONLY),
    DEFINE_REG_MEMORY_PTR(CANMORE_DBG_MCU_CONTROL_RELEASE_TYPE_OFFSET, &mcu_control_release_type, REGISTER_PERM_READ_ONLY),
    DEFINE_REG_EXEC_CALLBACK(CANMORE_DBG_MCU_CONTROL_REBOOT_MCU_OFFSET, reboot_mcu_cb, REGISTER_PERM_WRITE_ONLY),
};

static reg_mapped_server_page_def_t debug_server_pages[] = {
    // General Control Region
    DEFINE_PAGE_REG_MAPPED(CANMORE_DBG_MCU_CONTROL_PAGE_NUM, debug_server_mcu_control_regs),
    DEFINE_PAGE_UNIMPLEMENTED(CANMORE_DBG_VERSION_STRING_PAGE_NUM),  // To be filled in at startup

    // Safety Status Region
    // TODO To be implemented
};

static reg_mapped_server_inst_t debug_server_inst = {
    .page_array = debug_server_pages,
    .num_pages = sizeof(debug_server_pages)/sizeof(*debug_server_pages),
    .control_interface_mode = CANMORE_TITAN_CONTROL_INTERFACE_MODE_NORMAL,
};

// ========================================
// Exported Functions
// ========================================

void debug_init(reg_mapped_server_tx_func tx_func) {
    debug_server_inst.tx_func = tx_func;

    // Fill out version
    mcu_control_major_version = MAJOR_VERSION;
    mcu_control_minor_version = MINOR_VERSION;
    mcu_control_release_type = RELEASE_TYPE;
    debug_server_pages[CANMORE_DBG_VERSION_STRING_PAGE_NUM].page_type = PAGE_TYPE_MEMORY_MAPPED_BYTE;
    debug_server_pages[CANMORE_DBG_VERSION_STRING_PAGE_NUM].type.mem_mapped_byte.perm = REGISTER_PERM_READ_ONLY;
    debug_server_pages[CANMORE_DBG_VERSION_STRING_PAGE_NUM].type.mem_mapped_byte.base_addr = (uint8_t*) FULL_BUILD_TAG;
    debug_server_pages[CANMORE_DBG_VERSION_STRING_PAGE_NUM].type.mem_mapped_byte.size = strlen(FULL_BUILD_TAG) + 1;

    // Fill out board id
    pico_get_unique_board_id(&mcu_control_flash_id.id_pico);
}

void debug_process_message(uint8_t *msg_buffer, size_t len) {
    reg_mapped_server_handle_request(&debug_server_inst, msg_buffer, len);
    if (enter_bootloader_on_return) {
        safety_enter_bootloader();
    }
    if (reboot_mcu_on_return) {
        watchdog_reboot(0, 0, 0);
    }
}

#endif
