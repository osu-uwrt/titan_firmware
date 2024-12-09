#ifndef DRIVER__CANBUS_H_
#define DRIVER__CANBUS_H_

#include "hardware/spi.h"

#include <assert.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/**
 * @file driver/canbus.h
 *
 * @brief Driver for using the MCP251XFD CAN Controller for CAN bus communication.
 *
 * The CANmore protocol (see the specification header file for more information) is layered on top of the can bus
 * library to allow for MTUs greater than 8 to be sent over CAN. This is essential for reducing overhead for
 * transmitting XRCE-DDS messages. The `canbus_msg_` set of functions use the CANmore message type, allowing for this
 * expanded MTU. These calls are reserved for XRCE-DDS traffic only. For any auxiliary traffic on the bus, use the
 * `canbus_utility_` set of functions, which have the CAN default MTU of 8 bytes.
 */

// PICO_CONFIG: CAN_MCP251xFD_USE_EXTERNAL_INTERRUPT_CB, Configure external interrupts. Allows hooking of GPIO interrupts in conjunction with this driver, type=bool, default=0, group=driver_canbus
#ifndef CAN_MCP251XFD_USE_EXTERNAL_INTERRUPT_CB
#define CAN_MCP251XFD_USE_EXTERNAL_INTERRUPT_CB 0
#endif

// ========================================
// CAN Bus Control Functions
// ========================================

enum canbus_library_error_codes {
    CANBUS_LIBERR_DEVICE_MALFUNCTION = 1,
    CANBUS_LIBERR_COMM_MALFUNCTION = 2,
    CANBUS_LIBERR_RX_OVERFLOW = 3,
    CANBUS_LIBERR_UNEXPECTED_INTERRUPT = 4,
    CANBUS_LIBERR_TEF_OVERFLOW = 5
};

enum canbus_receive_error_codes {
    CANBUS_RECVERR_BUS_ERROR = 1,
    CANBUS_RECVERR_MSG_TX_FAILURE = 2,

    CANBUS_RECVERR_DECODE_ERROR_BASE = 100,
};

typedef union canbus_error_data {
    uint32_t raw;
    struct __attribute__((packed)) {
        uint16_t error_code;
        uint16_t error_line:15;
        uint8_t is_driver_error:1;  // If set to 1, error_code corresponds to a driver error code, rather than library
                                    // error code
    };
} canbus_error_data_t;
static_assert(sizeof(canbus_error_data_t) == sizeof(uint32_t), "Struct canbus_error_data did not pack properly");

typedef void (*canbus_receive_error_cb_t)(enum canbus_receive_error_codes);
typedef void (*canbus_internal_error_cb_t)(canbus_error_data_t error_data);

/**
 * @brief Initializes a CAN bus on the given PIO block on the second core.
 * This code will initialize the device as a CANmore client with the given `client_id`
 *
 * @param client_id The client_id to use for CAN communications
 * @return True on successful initialization, false on failure
 *
 * @note This code takes its initialization paremeters from the board pin definitions file as well as the robot
 * definitions file. If the board file does not have these pins defined, they must be defined using cmake compiler
 * declarations
 */
bool canbus_init(unsigned int client_id);

/**
 * @brief Deinitializes the CAN bus hardware on the chip.
 *
 * This disables all CAN communications
 */
void canbus_deinit(void);

/**
 * @brief Return if a valid CAN bus is detected (Heartbeat messages are receiving ACKs).
 */
bool canbus_check_online(void);

/**
 * @brief Sets the error field sent with the heartbeat message over CAN bus
 *
 * @param device_in_error_state True if device is in error state
 */
void canbus_set_device_in_error(bool device_in_error_state);

/**
 * @brief Tick any events which need to occur within can bus
 */
void canbus_tick(void);

/**
 * @brief Boolean if `canbus_init` and the chip hasn't been turned off with `canbus_disable`
 */
extern bool canbus_initialized;

/**
 * @brief Boolean if `canbus_init` has been called.
 */
extern bool canbus_setup;

#define CAN_MCP251XFD_EXTERNAL_INTERRUPT_PIN MCP2517FD_INT_PIN
#define CAN_MCP251XFD_EXTERNAL_INTERRUPT_EVENTS GPIO_IRQ_LEVEL_LOW

#if CAN_MCP251XFD_USE_EXTERNAL_INTERRUPT_CB
/**
 * @brief Notify mcp251xfd library of a gpio interrupt
 * Only enabled if CAN_MCP251XFD_USE_EXTERNAL_INTERRUPT_CB defined to 1.
 * Note that this MUST be configured before calling canbus_init
 *
 * @param gpio gpio pin provided to gpio callback
 * @param events gpio events provided to gpio callback
 */
void can_mcp251xfd_interrupt_cb(uint gpio, uint32_t events);
#endif

// ========================================
// Callback Setup
// ========================================

/**
 * @brief Set callback for receive errors on can bus.
 * This error can occur if invalid data is received on the CAN bus.
 *
 * @param callback Pointer to callback function or NULL to disable callback
 */
void canbus_set_receive_error_cb(canbus_receive_error_cb_t callback);

/**
 * @brief Set callback for internal errors on can bus.
 * This error should not occur during normal operation, but can occur if a fault occurs on the second core, or if the
 * first core is not processing data fast enough.
 *
 * @param callback Pointer to callback function or NULL to disable callback
 */
void canbus_set_internal_error_cb(canbus_internal_error_cb_t callback);

// ========================================
// CANmore Message Functions
// ========================================

/**
 * @brief Opens the canbus communication channel.
 * @attention This must be called before receiving or transmitting message frames.
 *
 * This clears the TX/RX fifo and begins receiving data.
 * @return true Successfully opened
 * @return false An error occurred (canbus offline, not initialized, etc.)
 */
bool canbus_msg_open(void);

/**
 * @brief Closes the canbus communication channel
 * This unsubscribes from received messages.
 * This is important so that the RX fifo won't raise an overflow fault when data is not being processed
 */
void canbus_msg_close(void);

/**
 * @brief Returns if a CANmore message is availble to be read.
 * @attention Requires `canbus_init` to be called
 * @attention Can not be used in interrupts
 */
bool canbus_msg_read_available(void);

/**
 * @brief Reads the next available CANmore message in the buffer.
 * @attention Requires `canbus_init` to be called
 * @attention Can not be used in interrupts
 *
 * @param buf The buffer for the received message data
 * @param len The buffer size, if the received message is longer than this value it will be truncated
 * @return The length of data copied into the buffer
 */
size_t canbus_msg_read(uint8_t *buf, size_t len);

/**
 * @brief Returns if space is available to write another CANmore message in the transmit buffer.
 * @attention Requires `canbus_init` to be called
 * @attention Can not be used in interrupts
 */
bool canbus_msg_write_available(void);

/**
 * @brief Queues the given buffer to sent with the CANmore message type over CAN.
 * @attention Requires `canbus_init` to be called
 * @attention Can not be used in interrupts
 *
 * @param buf The buffer containing data to transmit
 * @param len The length of data to transmit. If greater than `CANMORE_MAX_MSG_LENGTH` this will be truncated
 * @return The number of bytes transmitted (either `len` or `CANMORE_MAX_MSG_LENGTH` if truncated)
 */
size_t canbus_msg_write(const uint8_t *buf, size_t len);

// ========================================
// CANmore Utility Frame Functions
// ========================================

/**
 * @brief Returns if space is available to write another CANmore utility frame in the transmit buffer.
 * @attention Requires `canbus_init` to be called
 * @attention Can not be used in interrupts
 */
bool canbus_utility_frame_write_available(void);

/**
 * @brief Queues the given buffer to sent with the CANmore frame type over CAN.
 * @attention Requires `canbus_init` to be called
 * @attention Can not be used in interrupts
 *
 * @param channel The channel this utility frame is to be transmitted on
 * @param buf The buffer containing data to transmit
 * @param len The length of data to transmit. If greater than `CANMORE_MAX_FRAME_SIZE` this will be truncated
 * @return The number of bytes transmitted (either `len` or `CANMORE_MAX_FRAME_SIZE` if truncated)
 */
size_t canbus_utility_frame_write(uint32_t channel, uint8_t *buf, size_t len);

typedef void (*canbus_utility_chan_cb_t)(uint32_t channel, uint8_t *buf, size_t len);

/**
 * @brief Register utility frame callback
 *
 * Note that this is called during canbus_tick
 *
 * @param channel The channel to receive messages for
 * @param cb The function to receive callbacks for
 */
void canbus_utility_frame_register_cb(uint32_t channel, canbus_utility_chan_cb_t cb);

#endif
