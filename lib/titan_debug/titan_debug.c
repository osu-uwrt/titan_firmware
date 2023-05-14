#include "pico/unique_id.h"

#include "titan/debug.h"
#include "titan/safety.h"

// ========================================
// Bindings
// ========================================

static union {
    pico_unique_board_id_t id_pico;
    uint32_t id_word[2];
} mcu_control_flash_id;
static_assert(sizeof(mcu_control_flash_id.id_word) == PICO_UNIQUE_BOARD_ID_SIZE_BYTES, "Flash unique ID does not match expected size");

uint32_t canmore_debug_magic = 0x0DBAA1F0;
bool enter_bootloader_on_return = false;

static bool reboot_mcu_cb(__unused const struct reg_mapped_server_register_definition *reg, __unused bool is_write, __unused uint32_t *data_ptr) {
    enter_bootloader_on_return = true;
    return true;
}

// ========================================
// Register Map
// ========================================

static reg_mapped_server_register_def_t debug_server_mcu_control_regs[] = {
    DEFINE_REG_MEMORY_PTR(0, &canmore_debug_magic, REGISTER_PERM_READ_ONLY),
    DEFINE_REG_EXEC_CALLBACK(1, reboot_mcu_cb, REGISTER_PERM_WRITE_ONLY),
    DEFINE_REG_MEMORY_PTR(2, &mcu_control_flash_id.id_word[0], REGISTER_PERM_READ_ONLY),
    DEFINE_REG_MEMORY_PTR(3, &mcu_control_flash_id.id_word[1], REGISTER_PERM_READ_ONLY),

};

static reg_mapped_server_page_def_t debug_server_pages[] = {
    // General Control Region
    DEFINE_PAGE_REG_MAPPED(0, debug_server_mcu_control_regs),
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
    pico_get_unique_board_id(&mcu_control_flash_id.id_pico);
}

void debug_process_message(uint8_t *msg_buffer, size_t len) {
    reg_mapped_server_handle_request(&debug_server_inst, msg_buffer, len);
    if (enter_bootloader_on_return) {
        safety_enter_bootloader();
    }
}