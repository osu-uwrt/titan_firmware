#ifndef REG_MAPPED_CLIENT_LINUX_H
#define REG_MAPPED_CLIENT_LINUX_H

#include <stdbool.h>
#include <stdint.h>

#include "canmore_titan/reg_mapped_client.h"

typedef struct reg_mapped_client_linux_inst {
    unsigned int if_index;
    int socket;
    uint8_t client_id;
    uint8_t channel;
    reg_mapped_client_cfg_t client_cfg;
} reg_mapped_client_linux_inst_t;

/**
 * @brief Open new register mapped client
 *
 * @param inst Pointer to struct to hold instance data
 * @param if_index CAN interface index
 * @param client_id CANmore client ID to communicate over
 * @param channel CANmore channel that the register mapped server is bound to
 * @param timeout_ms Timeout for receives in milliseconds
 * @return true Client successfully opened (must be closed with reg_mapped_client_linux_close)
 * @return false Failed to open client
 */
bool reg_mapped_client_linux_open(reg_mapped_client_linux_inst_t *inst, unsigned int if_index, uint8_t client_id, uint8_t channel, uint32_t timeout_ms);

/**
 * @brief Closes register mapped client.
 * The client must be open when calling this function
 *
 * @param inst The client instance to close
 * @return true successfully closed client
 * @return false failed to close client
 */
bool reg_mapped_client_linux_close(reg_mapped_client_linux_inst_t *inst);

/**
 * @brief Read a register
 *
 * @param inst Client instance pointer
 * @param page Page to read
 * @param offset Register offset to read
 * @param data_out Pointer to write read data
 * @return REG_MAPPED_RESULT_SUCCESSFUL on success, other error code on failure
 */
int reg_mapped_client_linux_read_register(reg_mapped_client_linux_inst_t *inst, uint8_t page, uint8_t offset, uint32_t *data_out);

/**
 * @brief Write to a register
 *
 * @param inst Client instance pointer
 * @param page Page to write to
 * @param offset Register offset to write
 * @param data Data word to write into register
 * @return REG_MAPPED_RESULT_SUCCESSFUL on success, other error code on failure
 */
int reg_mapped_client_linux_write_register(reg_mapped_client_linux_inst_t *inst, uint8_t page, uint8_t offset, uint32_t data);

/**
 * @brief Read array of words from a page
 *
 * @note This function can only read to words in the same page
 *
 * @param inst Client instance pointer
 * @param page Page to read
 * @param offset_start Start offset to begin reading in page
 * @param data_array Array of words to read data into
 * @param num_words  Number of words to read
 * @return REG_MAPPED_RESULT_SUCCESSFUL on success, other error code on failure
 */
int reg_mapped_client_linux_read_array(reg_mapped_client_linux_inst_t *inst, uint8_t page, uint8_t offset_start,
                                        uint32_t *data_array, uint8_t num_words);

/**
 * @brief Write array of words to a page
 *
 * @note This function can only write to words in the same page
 *
 * @param inst Client instance pointer
 * @param page Page to write
 * @param offset_start Start offset to begin writing in page
 * @param data_array Array of words to write
 * @param num_words  Number of words to write
 * @return REG_MAPPED_RESULT_SUCCESSFUL on success, other error code on failure
 */
int reg_mapped_client_linux_write_array(reg_mapped_client_linux_inst_t *inst, uint8_t page, uint8_t offset_start,
                                        const uint32_t *data_array, uint8_t num_words);

#endif