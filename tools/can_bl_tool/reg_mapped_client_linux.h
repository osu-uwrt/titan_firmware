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
 * @brief Return pointer to reg mapped client to use with reg_mapped_client functions
 *
 * @note The returned reg_mapped_client instance will be invalid after calling reg_mapped_client_linux_close
 *
 * @param inst Instance to retrieve reg_mapped_client for
 * @return reg_mapped_client_cfg_t* Pointer to reg_mapped_client tied to this instance
 */
static inline reg_mapped_client_cfg_t* reg_mapped_client_linux_get_client(reg_mapped_client_linux_inst_t *inst) {
    return &inst->client_cfg;
}

#endif