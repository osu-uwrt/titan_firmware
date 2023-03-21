#ifndef CANMORE_TITAN__REG_MAPPED_SERVER_H
#define CANMORE_TITAN__REG_MAPPED_SERVER_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "canmore_titan/reg_mapped_protocol.h"

struct reg_mapped_server_register_definition;

// ========================================
// Register Map Structures
// ========================================

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
        REGISTER_TYPE_UNIMPLEMENTED = 0,
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
        PAGE_TYPE_UNIMPLEMENTED = 0,
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

// ========================================
// Convenience Macros for Register Defs
// ========================================
/**
 * @brief Create a register that is a pointer to memory
 *
 * @example
 *  #define EXAMPLE_PAGE_TEST_DATA_OFFSET 0
 *  uint32_t test_data = 0xDEADBEEF;
 *  const reg_mapped_server_register_def_t example_page[] = {
 *      DEFINE_REG_MEMORY_PTR(EXAMPLE_PAGE_TEST_DATA_OFFSET, &test_data, REGISTER_PERM_READ_ONLY)
 *  };
 */
#define DEFINE_REG_MEMORY_PTR(reg_offset, ptr, permission) [reg_offset] = {.reg_type = REGISTER_TYPE_MEMORY, .type = {.memory = {.perm = (permission), .reg_ptr = (ptr)}}}

/**
 * @brief Create a register that executes a callback
 *
 * @example
 *  #define EXAMPLE_PAGE_MY_CALLBACK_OFFSET 1
 *  bool my_callback(const struct reg_mapped_server_register_definition *reg, bool is_write, uint32_t *data_ptr) {
 *      // ... Do stuff here
 *  }
 *  const reg_mapped_server_register_def_t example_page[] = {
 *      DEFINE_REG_EXEC_CALLBACK(EXAMPLE_PAGE_MY_CALLBACK_OFFSET, my_callback, REGISTER_PERM_WRITE_ONLY)
 *  };
 */
#define DEFINE_REG_EXEC_CALLBACK(reg_offset, reg_callback, permission) [reg_offset] = {.reg_type = REGISTER_TYPE_EXEC, .type = {.exec = {.perm = (permission), .callback = (reg_callback)}}}

/**
 * @brief Create an unimplemented register
 *
 * @example
 *  #define EXAMPLE_PAGE_UNIMPLEMENTED_OFFSET 2
 *  const reg_mapped_server_register_def_t example_page[] = {
 *      DEFINE_REG_UNIMPLEMENTED(EXAMPLE_PAGE_UNIMPLEMENTED_OFFSET)
 *  };
 */
#define DEFINE_REG_UNIMPLEMENTED(reg_offset) [reg_offset] = {.reg_type = REGISTER_TYPE_UNIMPLEMENTED}

/**
 * @brief Create a register mapped page
 *
 * @example
 *  #define EXAMPLE_PAGE_NUM 0
 *  const reg_mapped_server_register_def_t example_page[] = {
 *      // ... Define your registers here
 *  };
 *
 *  const reg_mapped_server_page_def_t example_server_pages[] = {
 *      DEFINE_PAGE_REG_MAPPED(EXAMPLE_PAGE_NUM, example_page)
 *  };
 */
#define DEFINE_PAGE_REG_MAPPED(page_num, reg_map) [page_num] = {.page_type = PAGE_TYPE_REGISTER_MAPPED, .type = {.reg_mapped = {.num_registers = (sizeof(reg_map)/sizeof(*(reg_map))), .reg_array = reg_map}}}

/**
 * @brief Create a page memory mapped to a uint8_t* pointer
 *
 * @example
 *  #define PTR_MAPPED_PAGE_NUM 1
 *
 *  // These are defined by the application
 *  uint8_t *mapped_ptr;
 *  size_t mapped_size;
 *
 *  const reg_mapped_server_page_def_t example_server_pages[] = {
 *      DEFINE_PAGE_MEMMAPPED_BYTE_PTR(PTR_MAPPED_PAGE_NUM, mapped_ptr, mapped_size, REGISTER_PERM_READ_WRITE)
 *  };
 */
#define DEFINE_PAGE_MEMMAPPED_BYTE_PTR(page_num, ptr, len, permission) [page_num] = {.page_type = PAGE_TYPE_MEMORY_MAPPED_BYTE, .type = {.mem_mapped_byte = {.perm = (permission), .base_addr = (ptr), .size = (len)}}}

/**
 * @brief Create a page mapped to a uint8_t array
 *
 * @example
 *  #define ARRAY_MAPPED_PAGE_NUM 2
 *
 *  // These are defined by the application
 *  uint8_t test_buffer[] = {0x01, 0x23, 0x45, 0x67, 0x89, 0xAB};
 *
 *  const reg_mapped_server_page_def_t example_server_pages[] = {
 *      DEFINE_PAGE_MEMMAPPED_BYTE_ARRAY(ARRAY_MAPPED_PAGE_NUM, test_buffer, REGISTER_PERM_READ_WRITE)
 *  };
 */
#define DEFINE_PAGE_MEMMAPPED_BYTE_ARRAY(page_num, array, permission) DEFINE_PAGE_MEMMAPPED_BYTE_PTR(page_num, array, sizeof(array), permission)

/**
 * @brief Create an unimplemented page. This will error on usage
 *
 * @example
 *  #define UNIMPLEMENTED_PAGE_NUM 3
 *  const reg_mapped_server_page_def_t example_server_pages[] = {
 *      DEFINE_PAGE_UNIMPLEMENTED(UNIMPLEMENTED_PAGE_NUM)
 *  };
 */
#define DEFINE_PAGE_UNIMPLEMENTED(page_num) [page_num] = {.page_type = PAGE_TYPE_UNIMPLEMENTED}

// ========================================
// Functions
// ========================================

/**
 * @brief Handles a received request for the reg mapped server
 *
 * @param inst The reg mapped server instance holding state data
 * @param msg Received request
 * @param len Length of msg
 */
void reg_mapped_server_handle_request(reg_mapped_server_inst_t *inst, uint8_t *msg, size_t len);

#endif