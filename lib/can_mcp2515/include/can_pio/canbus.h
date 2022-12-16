#ifndef CAN_MULTICORE_H
#define CAN_MULTICORE_H

#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>

#include "hardware/spi.h"

/**
 * @file can_multicore.h
 *
 * API for creating a CAN bus controller built into the PIO block following the CANmore protocol
 *
 * This API uses the can2040 [github:KevinOConnor/can2040] library for creating the CAN bus in PIO. To keep IRQ latency
 * low on the CAN interface (a necessity for the can2040 library), this code uses the second core for all CAN bus code.
 *
 * The CANmore protocol (see the specification header file for more information) is layered on top of the can2040
 * library to allow for MTUs greater than 8 to be sent over CAN. This is essential for reducing overhead for
 * transmitting XRCE-DDS messages. The `canbus_msg_` set of functions use the CANmore message type, allowing for this
 * expanded MTU. These calls are reserved for XRCE-DDS traffic only. For any auxiliary traffic on the bus, use the
 * `canbus_utility_` set of functions, which have the CAN default MTU of 8 bytes.
*/


// ========================================
// CAN Bus Control Functions
// ========================================

typedef void (*canbus_cb_t)(void);
typedef void (*canbus_error_cb_t)(int);

/**
 * @brief Initializes a CAN bus on the given PIO block on the second core.
 * This code will initialize the device as a CANmore client with the given `client_id`
 *
 * @note This code will use the entire PIO block. The PIO block cannot be used for any other purpose
 * @note This code uses the second core, ensure that no other code is running on this core
 *
 * @param pio_num The PIO block to use to run the CAN bus
 * @param bitrate The CAN bus bitrate in bits/s
 * @param client_id The client ID for this node (See CANmore specification)
 * @param gpio_rx The GPIO pin to use for CAN RX
 * @param gpio_tx The GPIO pin to use for CAN TX
 * @param gpio_term The GPIO pin used for the termination resistor sense line, or -1 if not used
*/
void canbus_init(uint bitrate,
                 uint client_id,
                 spi_inst_t* spi_channel,
                 uint8_t cs_pin,
                 uint8_t mosi_pin,
                 uint8_t miso_pin,
                 uint8_t sck_pin,
                 uint32_t spi_clock,
                 uint8_t int_pin);

/**
 * @brief Return if a valid CAN bus is detected (Heartbeat messages are receiving ACKs).
*/
bool canbus_check_online(void);

/**
 * @brief Check if the second core running the canbus has died.
 *
 * @return true if the core was previously started and the last received heartbeat is stale
 */
bool canbus_core_dead(void);

/**
 * @brief Sets the error field sent with the heartbeat message over CAN bus
 *
 * @param device_in_error_state True if device is in error state
 */
void canbus_set_device_in_error(bool device_in_error_state);

/**
 * @brief Boolean if `canbus_init` has been called.
*/
extern bool canbus_initialized;


// ========================================
// Callback Setup
// ========================================

/**
 * @brief Set callback for receive errors on can bus.
 * This error can occur if invalid data is received on the CAN bus.
 *
 * @param callback Pointer to callback function or NULL to disable callback
 */
void canbus_set_receive_error_cb(canbus_error_cb_t callback);

/**
 * @brief Set callback for internal errors on can bus.
 * This error should not occur during normal operation, but can occur if a fault occurs on the second core, or if the
 * first core is not processing data fast enough.
 *
 * @param callback Pointer to callback function or NULL to disable callback
 */
void canbus_set_internal_error_cb(canbus_error_cb_t callback);

/**
 * @brief Set callback to notify that a utility frame has been received
 *
 * @param callback Pointer to callback function or NULL to disable callback
 */
void canbus_set_utility_frame_recv_cb(canbus_cb_t callback);

/**
 * @brief Set callback when a complete CANmore message has been received
 *
 * @param callback Pointer to callback function or NULL to disable callback
 */
void canbus_set_message_recv_cb(canbus_cb_t callback);


// ========================================
// CANmore Message Functions
// ========================================

/**
 * @brief The maximum length of a transmitted or received msg.
*/
extern const size_t canbus_msg_max_length;

/**
 * @brief Returns if a CANmore message is availble to be read.
 * @attention Requires `canbus_init` to be called
*/
bool canbus_msg_read_available(void);

/**
 * @brief Reads the next available CANmore message in the buffer.
 * @attention Requires `canbus_init` to be called
 *
 * @param buf The buffer for the received message data
 * @param len The buffer size, if the received message is longer than this value it will be truncated
 * @return The length of data copied into the buffer
*/
size_t canbus_msg_read(uint8_t *buf, size_t len);

/**
 * @brief Returns if space is available to write another CANmore message in the transmit buffer.
 * @attention Requires `canbus_init` to be called
*/
bool canbus_msg_write_available(void);

/**
 * @brief Queues the given buffer to sent with the CANmore message type over CAN.
 * @attention Requires `canbus_init` to be called
 *
 * @param buf The buffer containing data to transmit
 * @param len The length of data to transmit. If greater than `canbus_msg_max_length` this will be truncated
 * @return The number of bytes transmitted (either `len` or `canbus_msg_max_length` if truncated)
*/
size_t canbus_msg_write(const uint8_t *buf, size_t len);


// ========================================
// CANmore Utility Frame Functions
// ========================================

/**
 * @brief The maximum length of a transmitted or received utility frame.
*/
extern const size_t canbus_utility_frame_max_length;

/**
 * @brief Returns if a CANmore utility frame is availble to be read.
 * @attention Requires `canbus_init` to be called
*/
bool canbus_utility_frame_read_available(void);

/**
 * @brief Reads the next available CANmore utility frame in the buffer.
 * @attention Requires `canbus_init` to be called
 *
 * @param channel_out A pointer to write the channel this frame was received on
 * @param buf The buffer for the received frame data
 * @param len The buffer size, if the received frame is longer than this value it will be truncated
 * @return The length of data copied into the buffer
*/
size_t canbus_utility_frame_read(uint32_t *channel_out, uint8_t *buf, size_t len);

/**
 * @brief Returns if space is available to write another CANmore utility frame in the transmit buffer.
 * @attention Requires `canbus_init` to be called
*/
bool canbus_utility_frame_write_available(void);

/**
 * @brief Queues the given buffer to sent with the CANmore frame type over CAN.
 * @attention Requires `canbus_init` to be called
 *
 * @param channel The channel this utility frame is to be transmitted on
 * @param buf The buffer containing data to transmit
 * @param len The length of data to transmit. If greater than `canbus_utility_frame_max_length` this will be truncated
 * @return The number of bytes transmitted (either `len` or `canbus_utility_frame_max_length` if truncated)
*/
size_t canbus_utility_frame_write(uint32_t channel, uint8_t *buf, size_t len);

#endif