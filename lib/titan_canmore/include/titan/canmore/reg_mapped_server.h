#ifndef TITAN__CANMORE__REG_MAPPED_SERVER_H_
#define TITAN__CANMORE__REG_MAPPED_SERVER_H_

#include "titan/canmore/reg_mapped_protocol.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Register-Mapped Server Implementation
 * *************************************
 *
 * This file defines a register-mapped server which handles the heavy lifting of decoding the canmore register mapped
 * packets and provides several ways to expose "registers" over the canmore register-mapped protocol. This allows the
 * higher-level code to not have to worry about how the register mapped protocol works, and instead exposes a variety
 * of flexible interfaces for handling register accesses.
 *
 * Refer to reg_mapped_protocol.h to understand the theory of the register mapped protocol before continuing.
 *
 * Registers are broken into pages. Each page can be implemented in two ways:
 *  - A page as a contiguous region of memory (which can be read-only, write-only, or read-write)
 *    - This page can either be an array of 32-bit words
 *    - Or, it can be an array of 8-bit words. The server handles properly converting between words and bytes
 *    - The register mapped server transparently handles reading and writing to this memory region
 *  - A Register Mapped Page. This is the most flexible, as it allows individual registers to be be defined
 *
 * Each register defined by the server in a register mapped page can be one of two types:
 *  - A 32-bit word, which is written/read transparently by the register mapped server
 *  - Or a callback, which allows user code to handle reads/writes accordingly
 *
 * Each register can additionally have permissions set, enforcing read-only or write-only regisisters (for both memory
 * and callback based registers).
 *
 * The process for using this register mapped server library is as follows:
 *  - Create a reg_mapped_server_inst object:
 *    - Create each page you want to have exposed over the register mapped interface
 *      - For register mapped pages, create an array of reg_mapped_server_register_definition structs defined using
 *        the DEFINE_REG_* macros
 *      - For contiguous pages, create the regions of memory you want to expose as a page
 *    - Create an array of reg_mapped_server_page_definition structs using the DEFINE_PAGE_* macros
 *    - Create a reg_mapped_server_inst struct, passing a pointer to the page definition array above and number of
 *      of defined pages.
 *    - Define which control interface mode is assigned to this server instance (this must be unique and defined in
 *      the titan/canmore/protocol.h file to ensure that it does not overlap with other control interfaces. This
 *      prevents a message intended for a different mode from accidentally corrupting the current state of this server.
 *    - Create the transmit callback which will be called with the encoded register mapped response. This should send
 *      the encoded packet back over the network to the client.
 *  - Create a method to receive the register mapped protocol packets. Whenever a new packet is received for the
 *    register mapped protocol, it is passed to the reg_mapped_server_handle_request along with a handle to the register
 *    mapped server instance.
 */

struct reg_mapped_server_register_definition;

// ========================================
// Register Map Structures
// ========================================

/**
 * @brief Transmit function
 *
 * This is called by the reg mapped server handler code to send responses back to the client. No responses are returned
 * to the function who calls handle_request. Instead, this callback is used when handle_request is called.
 *
 * This must be filled out in the reg_mapped_server_inst_t before calling handle_request.
 */
typedef void (*reg_mapped_server_tx_func)(uint8_t *msg, size_t len);

/**
 * @brief Callback for exec type register accesses.
 *
 * This is implemented in the code creating the reg_mapped_server_register_definition, allowing for code to execute
 * when a specific register is written.
 *
 * @param reg The register referencing the callback
 * @param is_write True if register write, False if register read
 * @param data_ptr Contains the data written if is_write is true, or should be written to with the data to return if
 * is_write is false Note that this data will be sent back to the client, even if this function returns false
 * @return true, Reports successful execution
 * @return false, Error with invalid data
 */
typedef bool (*reg_mapped_server_register_cb_t)(const struct reg_mapped_server_register_definition *reg, bool is_write,
                                                uint32_t *data_ptr);

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
 *
 * These are combined together into an array to define a page.
 */
typedef struct reg_mapped_server_register_definition {
    enum reg_mapped_server_register_type {
        REGISTER_TYPE_UNIMPLEMENTED = 0,  // Unimplemented register, returns error when accessed
        REGISTER_TYPE_MEMORY,  // A register directly mapped to a 32-bit word. Access controlled by perm field
        REGISTER_TYPE_EXEC     // A register which fires a callback when accessed. The callback only fires if perm field
                               // allows that type of access
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
 *
 * These are combined together in an array to define the server instance.
 */
typedef struct reg_mapped_server_page_definition {
    enum reg_mapped_server_page_type {
        PAGE_TYPE_UNIMPLEMENTED = 0,   // Unimplemented page, returns error when accessed
        PAGE_TYPE_MEMORY_MAPPED_WORD,  // A page mapped directly to an array of 32-bit words
        PAGE_TYPE_MEMORY_MAPPED_BYTE,  // A page mapped directly to an array of bytes
        PAGE_TYPE_REGISTER_MAPPED      // A page backed by an array of register definitions
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
 * @brief Register mapped server instance declaration.
 * Passed to the register mapped server request handler to determine how to handle the packet.
 */
typedef struct reg_mapped_server_inst {
    // Must be populated by code calling reg_mapped_server_handle_request
    reg_mapped_server_tx_func tx_func;               // Function called by request handler to transmit responses
    const reg_mapped_server_page_def_t *page_array;  // Array of register mapped pages implemented by the server
    size_t num_pages;                                // Number of elements in page_array
    uint8_t control_interface_mode;  // The mode implemented by the reg mapped server. A packet's mode field must match
                                     // this value to be processed

    // Should be initialized to 0, holds state tracking data
    // This is noot to be modified by the caller of reg_mapped_server_handle_request
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
#define DEFINE_REG_MEMORY_PTR(reg_offset, ptr, permission)                                                             \
    [reg_offset] = { .reg_type = REGISTER_TYPE_MEMORY,                                                                 \
                     .type = { .memory = { .perm = (permission), .reg_ptr = (ptr) } } }

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
#define DEFINE_REG_EXEC_CALLBACK(reg_offset, reg_callback, permission)                                                 \
    [reg_offset] = { .reg_type = REGISTER_TYPE_EXEC,                                                                   \
                     .type = { .exec = { .perm = (permission), .callback = (reg_callback) } } }

/**
 * @brief Create an unimplemented register
 *
 * @example
 *  #define EXAMPLE_PAGE_UNIMPLEMENTED_OFFSET 2
 *  const reg_mapped_server_register_def_t example_page[] = {
 *      DEFINE_REG_UNIMPLEMENTED(EXAMPLE_PAGE_UNIMPLEMENTED_OFFSET)
 *  };
 */
#define DEFINE_REG_UNIMPLEMENTED(reg_offset) [reg_offset] = { .reg_type = REGISTER_TYPE_UNIMPLEMENTED }

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
#define DEFINE_PAGE_REG_MAPPED(page_num, reg_map)                                                                      \
    [page_num] = { .page_type = PAGE_TYPE_REGISTER_MAPPED,                                                             \
                   .type = { .reg_mapped = { .num_registers = (sizeof(reg_map) / sizeof(*(reg_map))),                  \
                                             .reg_array = reg_map } } }

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
#define DEFINE_PAGE_MEMMAPPED_BYTE_PTR(page_num, ptr, len, permission)                                                 \
    [page_num] = { .page_type = PAGE_TYPE_MEMORY_MAPPED_BYTE,                                                          \
                   .type = { .mem_mapped_byte = { .perm = (permission), .base_addr = (ptr), .size = (len) } } }

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
#define DEFINE_PAGE_MEMMAPPED_BYTE_ARRAY(page_num, array, permission)                                                  \
    DEFINE_PAGE_MEMMAPPED_BYTE_PTR(page_num, array, sizeof(array), permission)

/**
 * @brief Create an unimplemented page. This will error on usage
 *
 * @example
 *  #define UNIMPLEMENTED_PAGE_NUM 3
 *  const reg_mapped_server_page_def_t example_server_pages[] = {
 *      DEFINE_PAGE_UNIMPLEMENTED(UNIMPLEMENTED_PAGE_NUM)
 *  };
 */
#define DEFINE_PAGE_UNIMPLEMENTED(page_num) [page_num] = { .page_type = PAGE_TYPE_UNIMPLEMENTED }

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

#ifdef __cplusplus
}
#endif

#endif
