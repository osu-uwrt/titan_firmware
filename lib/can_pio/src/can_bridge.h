#ifndef CAN_BRIDGE_H
#define CAN_BRIDGE_H

#include "pico/time.h"
#include <stdint.h>
#include <stddef.h>

#include "basic_queue/queue.h"

#include "canmore_msg_encoding.h"

/**
 * @file can_bridge.h
 *
 * Contains required interface definitions for core-to-core communication
*/

// TODO: Add proper comments for preprocessor macros
#define CAN_MESSAGE_RX_QUEUE_SIZE 8
#define CAN_MESSAGE_TX_QUEUE_SIZE 4
#define CAN_UTILITY_RX_QUEUE_SIZE 8
#define CAN_UTILITY_TX_QUEUE_SIZE 4

#define CAN_HEARTBEAT_INTERVAL_MS 1000
// Timeout between last successful transmission before CAN bus appears to be offline. Must be greater than interval ms
#define CAN_HEARTBEAT_TIMEOUT_MS 1500

/**
 * @brief Flag sent from second core after successful initialization
 * After this is received from the SIO FIFO the second core is considered operational and ready to process data
*/
#define CORE1_STARTUP_DONE_FLAG 0xFEEDBAC1

/**
 * @brief Storage for initial configuration on second core, send at startup
*/
typedef struct can_init_cfg {
    uint pio_num;
    uint bitrate;
    uint client_id;
    uint gpio_rx;
    uint gpio_tx;
    int gpio_term;  // -1 if not used
} can_init_cfg_t;


/**
 * @brief Storage for queuing CANmore message data
*/
struct canmore_msg {
    // Message data buffer
    uint8_t data[CANMORE_MAX_MSG_LENGTH];
    // Length of data in buffer
    size_t length;
};

/**
 * @brief Storage for queuing CANmore utility frame data
*/
struct canmore_utility_frame {
    // Channel to use for utility frame
    uint8_t channel;
    // Frame DLC
    uint32_t dlc;
    // Data buffer
    union {
        uint8_t data[8];
        uint32_t data32[2];
    };
};

/**
 * @brief Queues for inter-core communication for message and utility frame TX/RX
*/
extern struct msg_receive_queue QUEUE_DEFINE(struct canmore_msg, CAN_MESSAGE_RX_QUEUE_SIZE) msg_receive_queue;
extern struct msg_transmit_queue QUEUE_DEFINE(struct canmore_msg, CAN_MESSAGE_TX_QUEUE_SIZE) msg_transmit_queue;
extern struct utility_receive_queue QUEUE_DEFINE(struct canmore_utility_frame, CAN_UTILITY_RX_QUEUE_SIZE) utility_receive_queue;
extern struct utility_transmit_queue QUEUE_DEFINE(struct canmore_utility_frame, CAN_UTILITY_TX_QUEUE_SIZE) utility_transmit_queue;

/**
 * @brief The time at which the can bus heartbeat is considered timed out, an the CAN bus is considered lost
 */
extern volatile absolute_time_t canbus_heartbeat_timeout;

/**
 * Entry point for second core
*/
extern void core1_entry();

#endif