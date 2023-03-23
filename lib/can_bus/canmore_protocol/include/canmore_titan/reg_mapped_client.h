#ifndef CANMORE_TITAN__REG_MAPPED_CLIENT_H
#define CANMORE_TITAN__REG_MAPPED_CLIENT_H

#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>

#include "canmore_titan/reg_mapped_protocol.h"

/**
 * @brief Transfer modes supported by client
 */
enum reg_mapped_client_transfer_mode {
    /**
     * @brief Only perform single register transfers
     */
    TRANSFER_MODE_SINGLE,
    /**
     * @brief Perform bulk transfers during array writes
     */
    TRNASFER_MODE_BULK,
    /**
     * @brief Perform multiword transfers during array read/writes
     */
    TRANSFER_MODE_MULTIWORD     // TODO: Implement multiword
};

/*
 * Additional error codes for reg_mapped_client
 */
#define REG_MAPPED_CLIENT_RESULT_TX_FAIL 0x100
#define REG_MAPPED_CLIENT_RESULT_RX_FAIL 0x101
#define REG_MAPPED_CLIENT_RESULT_RX_CLEAR_FAIL 0x102
#define REG_MAPPED_CLIENT_RESULT_INVALID_ARG 0x103
#define REG_MAPPED_CLIENT_RESULT_INVALID_BULK_COUNT 0x104

/**
 * @brief Transmit function prototype
 * Note that this function is responsible for determining which channel/client to communicate with
 * This data should be stored in arg, along with any additional required information (such as interface)
 *
 * @param buf The data to send
 * @param len Length of buffer to send
 * @param arg User argment for client
 * @return true If transmit successful
 * @return false If transmit failed
 */
typedef bool (*reg_mapped_client_tx_func)(const uint8_t *buf, size_t len, void* arg);

/**
 * @brief Clears the receive buffer
 *
 * @param arg User argument for client
 * @return true if clear successful
 * @return false if clear failed
 */
typedef bool (*reg_mapped_client_clear_rx_func)(void* arg);

/**
 * @brief Receive function prototype
 * Note that this function is responsible for determining which channel/client to communicate with
 * This data should be stored in arg, along with any additional required information (such as interface)
 *
 * @param buf Buffer to store received data
 * @param len Length of data to receive
 * @param timeout_ms Timeout for receive in ms
 * @param arg User argment for client
 * @return true If data was successfully received
 * @return false If an error occurred during receive, a timeout occurred, or received data did not match len
 */
typedef bool (*reg_mapped_client_rx_func)(uint8_t *buf, size_t len, unsigned int timeout_ms, void* arg);

/**
 * @brief Configuration struct for reg mapped client library functions
 */
typedef struct reg_mapped_client_cfg {
    /**
     * @brief Callback to perform transmit
     */
    reg_mapped_client_tx_func tx_func;
    /**
     * @brief Callback to clear receive buffer
     */
    reg_mapped_client_clear_rx_func clear_rx_func;
    /**
     * @brief Callback to perform receive
     */
    reg_mapped_client_rx_func rx_func;
    /**
     * @brief Optional argument data to pass to tx_func and rx_func
     */
    void* arg;
    /**
     * @brief Transfer mode to use for array transfers
     */
    enum reg_mapped_client_transfer_mode transfer_mode;
    /**
     * @brief Timeout to use for receive callback
     */
    unsigned int timeout_ms;
} reg_mapped_client_cfg_t;

/**
 * @brief Read a register
 *
 * @param cfg Register mapped client configuration
 * @param page Page to read
 * @param offset Register offset to read
 * @param data_out Pointer to write read data
 * @return REG_MAPPED_RESULT_SUCCESSFUL on success, other error code on failure
 */
int reg_mapped_client_read_register(const reg_mapped_client_cfg_t *cfg, uint8_t page, uint8_t offset, uint32_t *data_out);

/**
 * @brief Write to a register
 *
 * @param cfg Register mapped client configuration
 * @param page Page to write to
 * @param offset Register offset to write
 * @param data Data word to write into register
 * @return REG_MAPPED_RESULT_SUCCESSFUL on success, other error code on failure
 */
int reg_mapped_client_write_register(const reg_mapped_client_cfg_t *cfg, uint8_t page, uint8_t offset, uint32_t data);

/**
 * @brief Read array of words from a page
 *
 * @note This function can only read to words in the same page
 *
 * @param cfg Register mapped client configuration
 * @param page Page to read
 * @param offset_start Start offset to begin reading in page
 * @param data_array Array of words to read data into
 * @param num_words  Number of words to read
 * @return REG_MAPPED_RESULT_SUCCESSFUL on success, other error code on failure
 */
int reg_mapped_client_read_array(const reg_mapped_client_cfg_t *cfg, uint8_t page, uint8_t offset_start,
                                    uint32_t *data_array, uint8_t num_words);

/**
 * @brief Write array of words to a page
 *
 * @note This function can only write to words in the same page
 *
 * @param cfg Register mapped client configuration
 * @param page Page to write
 * @param offset_start Start offset to begin writing in page
 * @param data_array Array of words to write
 * @param num_words  Number of words to write
 * @return REG_MAPPED_RESULT_SUCCESSFUL on success, other error code on failure
 */
int reg_mapped_client_write_array(const reg_mapped_client_cfg_t *cfg, uint8_t page, uint8_t offset_start,
                                    const uint32_t *data_array, uint8_t num_words);

/**
 * @brief Read a page containing a null-temrinated string
 *
 * @param cfg Register mapped client configuration
 * @param page_num Page containing null-terminated string
 * @param str_out Pointer to store received string
 * @param max_len Max number of bytes to write to str_out
 * @return int Length of read string, or negative on error
 */
int reg_mapped_client_read_string_page(const reg_mapped_client_cfg_t *cfg, uint8_t page_num,
                                        char* str_out, size_t max_len);

#endif