#ifndef CANMORE_TITAN__REG_MAPPED_SERVER_H
#define CANMORE_TITAN__REG_MAPPED_SERVER_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "canmore_titan/reg_mapped_protocol.h"

struct reg_mapped_server_register_definition;

/**
 * @brief Transmit function
 */
typedef void (*reg_mapped_server_tx_func)(uint8_t *msg, size_t len);

/**
 * @brief Callback for exec type register write
 *
 * @param reg The register referencing the callback
 * @param is_write True if register write, False if register read
 * @param data_ptr Contains the data written if is_write is true, or should be written to with the data to return if is_write is false
 *                 Note that this data will be sent back to the client, even if this function returns false
 * @return true, Reports successful execution
 * @return false, Error with invalid data
 */
typedef bool (*reg_mapped_server_register_cb_t)(const struct reg_mapped_server_register_definition *reg, bool is_write, uint32_t *data_ptr);

/**
 * @brief Valid Register Permissions for Register Mapped Server Memory Map
 */
enum reg_mapped_server_register_permissions {
    REGISTER_PERM_READ_WRITE,
    REGISTER_PERM_READ_ONLY,
    REGISTER_PERM_WRITE_ONLY
};

/**
 * @brief Individual Register Definition for Register Mapped Server Memory Map
 */
typedef struct reg_mapped_server_register_definition {
    enum reg_mapped_server_register_type {
        REGISTER_TYPE_UNIMPLEMENTED,
        REGISTER_TYPE_MEMORY,
        REGISTER_TYPE_EXEC
    } reg_type;

    union {
        struct {
            enum reg_mapped_server_register_permissions perm;
            uint32_t *reg_ptr;
        } memory;

        struct {
            enum reg_mapped_server_register_permissions perm;
            reg_mapped_server_register_cb_t callback;
        } exec;
    } type;

} reg_mapped_server_register_def_t;

/**
 * @brief Individual Page Definition for Register Mapped Server Memory Map
 */
typedef struct reg_mapped_server_page_definition {
    enum reg_mapped_server_page_type {
        PAGE_TYPE_UNIMPLEMENTED,
        PAGE_TYPE_MEMORY_MAPPED_WORD,
        PAGE_TYPE_MEMORY_MAPPED_BYTE,
        PAGE_TYPE_REGISTER_MAPPED
    } page_type;

    union {
        struct {
            enum reg_mapped_server_register_permissions perm;
            uint32_t *base_addr;
            size_t num_words;
        } mem_mapped_word;

        struct {
            enum reg_mapped_server_register_permissions perm;
            uint8_t *base_addr;
            size_t size;
        } mem_mapped_byte;

        struct {
            size_t num_registers;
            const reg_mapped_server_register_def_t *reg_array;
        } reg_mapped;
    } type;
} reg_mapped_server_page_def_t;

/**
 * @brief Map of registers in
 */
typedef struct reg_mapped_server_register_map {
} server_register_map_t;

typedef struct reg_mapped_server_inst {
    // Must be populated by code calling reg_mapped_server_handle_request
    reg_mapped_server_tx_func tx_func;
    const reg_mapped_server_page_def_t *page_array;
    size_t num_pages;

    // Should be initialized to 0, holds state tracking data
    // ... Put stuff here
    uint8_t bulk_last_seq_num;
    uint8_t bulk_error_code;
    bool in_bulk_request;
} reg_mapped_server_inst_t;

/**
 * @brief Handles a received request for the reg mapped server
 *
 * @param inst The reg mapped server instance holding state data
 * @param msg Received request
 * @param len Length of msg
 */
void reg_mapped_server_handle_request(reg_mapped_server_inst_t *inst, uint8_t *msg, size_t len);

#endif