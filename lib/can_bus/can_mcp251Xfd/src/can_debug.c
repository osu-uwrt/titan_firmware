#include "pico/unique_id.h"

#include "canmore_titan/protocol.h"
#include "canmore_titan/reg_mapped_server.h"
#include "safety/safety.h"

#include "can_mcp251Xfd/canbus.h"
#include "can_mcp251XFD_bridge.h"

// ========================================
// Bindings
// ========================================

void can_debug_transmit(uint8_t *msg, size_t len) {
    canbus_utility_frame_write(CANMORE_TITAN_CHAN_CONTROL_INTERFACE, msg, len);
}

static union {
    pico_unique_board_id_t id_pico;
    uint32_t id_word[2];
} mcu_control_flash_id;
static_assert(sizeof(mcu_control_flash_id.id_word) == PICO_UNIQUE_BOARD_ID_SIZE_BYTES, "Flash unique ID does not match expected size");

uint32_t can_debug_magic = 0x0DBAA1F0;
bool enter_bootloader_on_return = false;

static bool reboot_mcu_cb(__unused const struct reg_mapped_server_register_definition *reg, __unused bool is_write, __unused uint32_t *data_ptr) {
    enter_bootloader_on_return = true;
    return true;
}

// ========================================
// Register Map
// ========================================

static reg_mapped_server_register_def_t debug_server_mcu_control_regs[] = {
    DEFINE_REG_MEMORY_PTR(0, &can_debug_magic, REGISTER_PERM_READ_ONLY),
    DEFINE_REG_EXEC_CALLBACK(1, reboot_mcu_cb, REGISTER_PERM_WRITE_ONLY),
    DEFINE_REG_MEMORY_PTR(2, &mcu_control_flash_id.id_word[0], REGISTER_PERM_READ_ONLY),
    DEFINE_REG_MEMORY_PTR(3, &mcu_control_flash_id.id_word[1], REGISTER_PERM_READ_ONLY),

};

static reg_mapped_server_page_def_t debug_server_pages[] = {
    // General Control Region
    DEFINE_PAGE_REG_MAPPED(0, debug_server_mcu_control_regs),
};

static reg_mapped_server_inst_t debug_server_inst = {
    .tx_func = &can_debug_transmit,
    .page_array = debug_server_pages,
    .num_pages = sizeof(debug_server_pages)/sizeof(*debug_server_pages),
    .control_interface_mode = CANMORE_TITAN_CONTROL_INTERFACE_MODE_NORMAL,
};

// ========================================
// Exported Functions
// ========================================

static uint8_t msg_buffer[REG_MAPPED_MAX_REQUEST_SIZE];

void can_debug_init(void) {
    pico_get_unique_board_id(&mcu_control_flash_id.id_pico);
}

void can_debug_tick(void) {
    // TODO: Make this not take the entire utility channel
    if (canbus_utility_frame_read_available()) {
        uint32_t channel;
        size_t len = canbus_utility_frame_read(&channel, msg_buffer, sizeof(msg_buffer));
        if (channel == CANMORE_TITAN_CHAN_CONTROL_INTERFACE) {
            reg_mapped_server_handle_request(&debug_server_inst, msg_buffer, len);
            if (enter_bootloader_on_return) {
                safety_enter_bootloader();
            }
        }
    }
}