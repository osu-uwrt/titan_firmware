#include <string.h>
#include "canmore_titan/protocol.h"
#include "canmore_titan/reg_mapped_server.h"

#include "bl_interface.h"
#include "bl_server.h"

#define COUNTOF(arr) (sizeof(arr)/sizeof(*(arr)))

uint32_t test_data = 0xDEADBEEF;
uint8_t test_range[] = {0x01, 0x23, 0x45, 0x67, 0x89, 0xAB};

const reg_mapped_server_register_def_t bl_server_control_page_regs[] = {
    {.reg_type = REGISTER_TYPE_MEMORY, .type = {.memory = {.perm = REGISTER_PERM_READ_ONLY, .reg_ptr = &test_data}}}
};

const reg_mapped_server_page_def_t bl_server_pages[] = {
    {.page_type = PAGE_TYPE_REGISTER_MAPPED, .type = {.reg_mapped = {.num_registers = COUNTOF(bl_server_control_page_regs), .reg_array = bl_server_control_page_regs}}},
    {.page_type = PAGE_TYPE_MEMORY_MAPPED_BYTE, .type = {.mem_mapped_byte = {.perm = REGISTER_PERM_READ_WRITE, .base_addr = test_range, .size = sizeof(test_range)}}},
};

reg_mapped_server_inst_t bl_server_inst = {
    .tx_func = &bl_interface_transmit,
    .page_array = bl_server_pages,
    .num_pages = COUNTOF(bl_server_pages)
};

static uint8_t msg_buffer[REG_MAPPED_MAX_REQUEST_SIZE];

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