#ifndef TRANSPORT_CAN_H
#define TRANSPORT_CAN_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define TIMEOUT_MS 1000

typedef struct {
    uint8_t client_id;

    uint8_t err_code;
    int32_t timeout;
} can_transport_data_t;

/**
 * @brief Initialize Micro-Ros using the CAN Transport
 * This handles the initialization of the CAN bus hardware
 *
 * @param client_id The client id to use for CAN traffic
 * @return true CAN hardware was successfully initialized and the transport was set
 * @return false CAN hardware failed to initialize
 */
bool transport_can_init(void *ctx);
size_t transport_can_read(uint8_t *buf, size_t len, void *ctx);
size_t transport_can_write(const uint8_t *buf, size_t len, void *ctx);
bool transport_can_close();
bool transport_can_open();

#endif
