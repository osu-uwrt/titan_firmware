#ifndef CAN_BRIDGE_H
#define CAN_BRIDGE_H

#include "pico/time.h"
#include <stdint.h>
#include <stddef.h>

#include "basic_queue/queue.h"

#include "canmore/msg_encoding.h"

/**
 * @file can_bridge.h
 *
 * Contains required interface definitions for core-to-core communication
*/

// ========================================
// Configuration
// ========================================

// PICO_CONFIG: CAN_MESSAGE_RX_QUEUE_SIZE, Queue size for CANmore messages received and decoded by the second (dedicated CAN) core waiting to be processed by the primary core, type=int, default=8, group=can_pio
#ifndef CAN_MESSAGE_RX_QUEUE_SIZE
#define CAN_MESSAGE_RX_QUEUE_SIZE 8
#endif

// PICO_CONFIG: CAN_MESSAGE_TX_QUEUE_SIZE, Queue size for CANmore messages generated by the primary core waiting to be transmitted by the second (dedicated CAN) core, type=int, default=4, group=can_pio
#ifndef CAN_MESSAGE_TX_QUEUE_SIZE
#define CAN_MESSAGE_TX_QUEUE_SIZE 4
#endif

// PICO_CONFIG: CAN_UTILITY_RX_QUEUE_SIZE, Queue size for CANmore utility frames received and decoded by the second (dedicated CAN) core waiting to be processed by the primary core, type=int, default=8, group=can_pio
#ifndef CAN_UTILITY_RX_QUEUE_SIZE
#define CAN_UTILITY_RX_QUEUE_SIZE 8
#endif

// PICO_CONFIG: CAN_UTILITY_TX_QUEUE_SIZE, Queue size for CANmore utility frames generated by the primary core waiting to be transmitted by the second (dedicated CAN) core, type=int, default=4, group=can_pio
#ifndef CAN_UTILITY_TX_QUEUE_SIZE
#define CAN_UTILITY_TX_QUEUE_SIZE 4
#endif

// PICO_CONFIG: CAN_FRAME_RX_QUEUE_SIZE, Queue size for CAN frames received by the can2040 interrupt waiting to be decoded in the main thread on the second (dedicated CAN) core, type=int, default=16, group=can_pio
#ifndef CAN_FRAME_RX_QUEUE_SIZE
#define CAN_FRAME_RX_QUEUE_SIZE 16
#endif

// PICO_CONFIG: CAN_CORE_ALIVE_INTERVAL_MS, Interval for SIO_CORE_ALIVE_FLAG to be sent in milliseconds, type=int, default=10, group=can_pio
#ifndef CAN_CORE_ALIVE_INTERVAL_MS
#define CAN_CORE_ALIVE_INTERVAL_MS 10
#endif

// PICO_CONFIG: CAN_CORE_ALIVE_TIMEOUT_MS, Timeout in milliseconds since last SIO_CORE_ALIVE_FLAG when the second (dedicated CAN) core is considered dead, type=int, default=15, group=can_pio
#ifndef CAN_CORE_ALIVE_TIMEOUT_MS
#define CAN_CORE_ALIVE_TIMEOUT_MS 15
#endif

// PICO_CONFIG: CAN_HEARTBEAT_INTERVAL_MS, Interval for CANmore heartbeat transmission over CAN bus in milliseconds, type=int, default=1000, group=can_pio
#ifndef CAN_HEARTBEAT_INTERVAL_MS
#define CAN_HEARTBEAT_INTERVAL_MS 1000
#endif

// PICO_CONFIG: CAN_HEARTBEAT_TIMEOUT_MS, Timeout since last successful ACK of heartbeat frame in milliseconds before CAN bus is considered lost. Must be greater than CAN_HEARTBEAT_INTERVAL_MS, type=int, default=1500, group=can_pio
#ifndef CAN_HEARTBEAT_TIMEOUT_MS
#define CAN_HEARTBEAT_TIMEOUT_MS 1500
#endif


// ========================================
// Core Bringup
// ========================================

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
 * @brief Entry point for second core
*/
extern void core1_entry();


// ========================================
// Heartbeat/Error Reporting
// ========================================

/**
 * @brief Flag sent from second core after successful initialization
 * After this is received from the SIO FIFO the second core is considered operational and ready to process data
*/
#define SIO_STARTUP_DONE_FLAG 0x0FEEDBAC

/**
 * @brief Flag sent from second core when an internal error occurs.
 * This error should not occur during normal operation, but can occur if a fault occurs on the second core, or if the
 * first core is not processing data fast enough.
 */
#define SIO_ERROR_INTERNAL_FLAG 0xF0000000

#define HAS_ERROR_INTERNAL_FLAG(value) ((value & 0xF0000000) == SIO_ERROR_INTERNAL_FLAG)

/**
 * @brief Flag sent from second core when a receive error occurs.
 * This error can occur if invalid data is received on the CAN bus.
 */
#define SIO_ERROR_RECEIVE_FLAG 0xE0000000

#define HAS_ERROR_RECEIVE_FLAG(value) ((value & 0xF0000000) == SIO_ERROR_RECEIVE_FLAG)

/**
 * @brief Flag sent from second core when a utility frame has been put into the receive buffer
 */
#define SIO_RECV_UTILITY_FRAME_FLAG 0xC0000001

/**
 * @brief Flag sent from second core when a CANMore message has been put into the receive buffer
 */
#define SIO_RECV_MESSAGE_FLAG 0xC0000002

/**
 * @brief Flag sent from second core to report it is still alive to the first core.
 * If this is not sent within a given interval, the core can be considered dead
 */
#define SIO_CORE_ALIVE_FLAG 0xAA0F00AA

/**
 * @brief The time at which the can bus heartbeat is considered timed out, an the CAN bus is considered lost
 */
extern volatile absolute_time_t canbus_heartbeat_timeout;

/**
 * @brief Boolean which controls error field of heartbeat messages
 */
extern volatile bool canbus_device_in_error_state;

// ========================================
// CAN Message/Frame Queues
// ========================================

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

#endif