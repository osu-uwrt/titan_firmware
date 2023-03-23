#include <stdio.h>

#include "canmore_titan/bootloader_interface.h"
#include "canmore_titan/reg_mapped_client.h"
#include "bl_client.h"

#define EXPECTED_VERSION_MAJOR 1
#define EXPECTED_VERSION_MINOR 0

// Defines pulled from board_version.h
#define VERSION_PROTO 0
#define VERSION_DEV 1
#define VERSION_STABLE 2

#define verbose 1

#define ret_client_check(client_result) do { \
    int result = client_result; \
    if ((result) != REG_MAPPED_RESULT_SUCCESSFUL) { \
        if (verbose) \
            printf("[" __FILE__ ":%d] Client request failed: 0x%x\n", __LINE__, result); \
        return false; \
    }} while(0)

bool bl_client_init(bl_client_inst_t *inst, reg_mapped_client_cfg_t* client) {
    inst->client = client;

    uint32_t bl_magic;
    ret_client_check(reg_mapped_client_read_register(client, CANMORE_BL_MCU_CONTROL_PAGE_NUM, CANMORE_BL_MCU_CONTROL_MAGIC_OFFSET, &bl_magic));

    if (bl_magic != CANMORE_BL_MCU_CONTROL_MAGIC_VALUE) {
        if (verbose) printf("Unexpected magic 0x%08x, expected 0x%08x\n", bl_magic, CANMORE_BL_MCU_CONTROL_MAGIC_VALUE);
        return false;
    }

    uint32_t version_major;
    uint32_t version_minor;
    uint32_t version_type;
    ret_client_check(reg_mapped_client_read_register(client, CANMORE_BL_MCU_CONTROL_PAGE_NUM,
                                                        CANMORE_BL_MCU_CONTROL_MAJOR_VERSION_OFFSET, &version_major));
    ret_client_check(reg_mapped_client_read_register(client, CANMORE_BL_MCU_CONTROL_PAGE_NUM,
                                                        CANMORE_BL_MCU_CONTROL_MINOR_VERSION_OFFSET, &version_minor));
    ret_client_check(reg_mapped_client_read_register(client, CANMORE_BL_MCU_CONTROL_PAGE_NUM,
                                                        CANMORE_BL_MCU_CONTROL_RELEASE_TYPE_OFFSET, &version_type));

    if (version_major != EXPECTED_VERSION_MAJOR || version_minor != EXPECTED_VERSION_MINOR) {
        printf("Warning! Unexpected version %d.%d - This program expects version %d.%d\n",
                version_major, version_minor, EXPECTED_VERSION_MAJOR, EXPECTED_VERSION_MINOR);
    }

    if (version_type == VERSION_PROTO) {
        printf("Warning! Bootloader running prototype firmware! This may not operate as expected\n");
    } else if (version_type == VERSION_DEV) {
        printf("Note: Target bootloader is a development version\n");
    } else if (version_type != VERSION_STABLE) {
        if (verbose) printf("Invalid version type %d\n", version_type);
        return false;
    }

    ret_client_check(reg_mapped_client_read_register(client, CANMORE_BL_MCU_CONTROL_PAGE_NUM,
                                                        CANMORE_BL_MCU_CONTROL_LOWER_FLASH_ID, &inst->flash_id.word[0]));
    ret_client_check(reg_mapped_client_read_register(client, CANMORE_BL_MCU_CONTROL_PAGE_NUM,
                                                        CANMORE_BL_MCU_CONTROL_UPPER_FLASH_ID, &inst->flash_id.word[1]));

    return true;
}

int bl_client_get_version(bl_client_inst_t *inst, char* version_out, size_t max_len) {
    return reg_mapped_client_read_string_page(inst->client, CANMORE_BL_VERSION_STRING_PAGE_NUM, version_out, max_len);
}

bool bl_client_reboot_server(bl_client_inst_t *inst) {
    ret_client_check(reg_mapped_client_write_register(inst->client, CANMORE_BL_MCU_CONTROL_PAGE_NUM,
                                                        CANMORE_BL_MCU_CONTROL_REBOOT_MCU_OFFSET, 1));

    return true;
}