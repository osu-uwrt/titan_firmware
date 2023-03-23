#ifndef BL_CLIENT_H
#define BL_CLIENT_H

#include <stdbool.h>
#include <stdint.h>

#include "canmore_titan/reg_mapped_client.h"

typedef struct bl_client_inst {
    reg_mapped_client_cfg_t* client;
    union {
        uint64_t doubleword;
        uint32_t word[2];
        uint8_t byte[8];
    } flash_id;
} bl_client_inst_t;

/**
 * @brief Open new bootloader client and attempt to establish a connection.
 *
 * The client argument is expected to be a properly initialized register mapped client.
 * This can created using a variety of methods (depending on communication channel)
 *
 * @param inst Pointer to struct holding instance data
 * @param client The reg mapped client configuration to use during communication
 * @return true Client successfully established connection
 * @return false Failed to initialize client
 */
bool bl_client_init(bl_client_inst_t *inst, reg_mapped_client_cfg_t* client);

/**
 * @brief Retrieves version string from the device
 *
 * @param inst bl_client instance
 * @param version_out Pointer to store received version string
 * @param max_len Max number of bytes to write to version_out
 * @return Length of read string, or negative on error
 */
int bl_client_get_version(bl_client_inst_t *inst, char* version_out, size_t max_len);

/**
 * @brief Restarts the connected bootloader server
 *
 * @param inst bl_client instance
 * @return true Successfully sent reboot command
 * @return false Failed to reboot bootloader server
 */
bool bl_client_reboot_server(bl_client_inst_t *inst);

/**
 * @brief Get the flash id from the device
 *
 * @param inst bl_client instance
 * @return uint64_t The 64-bit unique flash id
 */
static inline uint64_t bl_client_get_flash_id(bl_client_inst_t *inst) {
    return inst->flash_id.doubleword;
}

#endif