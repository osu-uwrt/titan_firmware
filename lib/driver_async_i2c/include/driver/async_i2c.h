#ifndef DRIVER__ASYNC_I2C_H_
#define DRIVER__ASYNC_I2C_H_

#include <stdint.h>
#include <stdbool.h>

#include "hardware/i2c.h"

/**
 * @file driver/async_i2c.h
 *
 * @brief Driver to allow asynchronous scheduling of i2c traffic
 *
 * Commands can be built using the async_i2c_request struct, and scheduled to be transmitted with async_i2c_enqueue. One
 * of the tqo possible callbacks for failure or completion will be called in the future. Each command consists of an
 * optional transmit and receive phase, where one or both of the phases can be populated.
 *
 * Convenience functions exist to allow previously written blocking code to use this library without changing the
 * code flow.
 *
 * @attention hardware_i2c calls should not be used on a bus initialized with Aync I2C.
 */

// PICO_CONFIG: PARAM_ASSERTIONS_ENABLED_ASYNC_I2C, Enable/disable assertions in the Async I2C module, type=bool, default=0, group=driver_async_i2c
#ifndef PARAM_ASSERTIONS_ENABLED_ASYNC_I2C
#define PARAM_ASSERTIONS_ENABLED_ASYNC_I2C 0
#endif

// PICO_CONFIG: I2C_REQ_QUEUE_SIZE, Number of pending i2c requests in the async_i2c request queue, type=int, default=16, group=driver_async_i2c
#ifndef I2C_REQ_QUEUE_SIZE
#define I2C_REQ_QUEUE_SIZE 16
#endif

// I2C Request Types
struct async_i2c_request;

/**
 * @brief Typedef for callback to i2c function
 */
typedef void (*async_i2c_cb_t)(const struct async_i2c_request *);
typedef void (*async_i2c_abort_cb_t)(const struct async_i2c_request *, uint32_t);
struct async_i2c_request {
    uint8_t i2c_num;                                        // I2C hardware number
    uint8_t address;                                        // I2C Address
    bool nostop;                                            // Sets nostop field in pico i2c requests
    const uint8_t *tx_buffer;                               // Buffer to transmit (must be at least bytes_to_send size)
    uint8_t *rx_buffer;                                     // Buffer to receive (must be at least bytes_to_receive size)
    uint16_t bytes_to_send;                                 // Number of bytes to transmit
    uint16_t bytes_to_receive;                              // Number of bytes to receive (receive phase occurs after transmit)
    async_i2c_cb_t completed_callback;                      // Callback upon successful completion of request (can be NULL)
    async_i2c_abort_cb_t failed_callback;                   // Callback upon failure of request (can be NULL)
    const struct async_i2c_request *next_req_on_success;    // Next request to send upon succesful completion of request (can be NULL if no chaining required)
    void* user_data;                                        // User data to access in callback
    absolute_time_t timeout;                                // Timeout for request to end, or nil_time if it should default to normal timeout
};

/**
 * @brief Creates a read/write request.
 * First sends tx_size bytes from tx_buf and reads rx_size bytes into rx_buf.
 * tx_buf and rx_buf must not be modified while in_progress is true
 * fn_callback is called on a successful request
 */
#define ASYNC_I2C_READ_WRITE_REQ(i2c_inst, target_address, tx_buf, rx_buf, tx_size, rx_size, fn_callback) {\
    .i2c = i2c_inst, \
    .address = target_address, \
    .nostop = false, \
    .tx_buffer = tx_buf, \
    .rx_buffer = rx_buf, \
    .bytes_to_send = tx_size, \
    .bytes_to_receive = rx_size, \
    .completed_callback = fn_callback, \
    .failed_callback = NULL, \
    .next_req_on_success = NULL, \
    .user_data = NULL}

/**
 * @brief Creates a write request
 * Sends tx_size bytes from tx_buf
 * tx_buf must not be modified while in_progress is true
 * fn_callback is called on a successful request
 */
#define ASYNC_I2C_WRITE_REQ(hw_id, address, tx_buf, tx_size, fn_callback) {\
    .i2c = i2c_inst, \
    .address = target_address, \
    .nostop = false, \
    .tx_buffer = tx_buf, \
    .rx_buffer = NULL, \
    .bytes_to_send = tx_size, \
    .bytes_to_receive = 0, \
    .completed_callback = fn_callback, \
    .failed_callback = NULL, \
    .next_req_on_success = NULL \
    .user_data = NULL}

/**
 * @brief Creates a read request
 * Reads rx_size bytes into rx_buf.
 * rx_buf must not be modified while in_progress is true
 * fn_callback is called on a successful request
 */
#define ASYNC_I2C_READ_REQ(i2c_inst, target_address, rx_buf, rx_size, fn_callback) {\
    .i2c = i2c_inst, \
    .address = target_address, \
    .nostop = false, \
    .tx_buffer = NULL, \
    .rx_buffer = rx_buf, \
    .bytes_to_send = 0, \
    .bytes_to_receive = rx_size, \
    .completed_callback = fn_callback, \
    .failed_callback = NULL, \
    .next_req_on_success = NULL \
    .user_data = NULL}



/**
 * @brief Bool if async_i2c_init has been called
 */
extern bool async_i2c_initialized;

/**
 * @brief Queues an async i2c request.
 * This function is safe to be called from interrupts, and from async_i2c callbacks
 *
 * @attention async_i2c_init must be called before this function
 *
 * @param request Struct if request is true
 * @param in_progress Pointer to be true while request is in progress (and buffers should not be modified)
 * Note that in_progress will be true while next_req_on_success is being processed as well
 * @return true if the request was successfully queued
 * @return false if the request failed to queue due to a full buffer
 */
bool async_i2c_enqueue(const struct async_i2c_request *request, volatile bool *in_progress);

/**
 * @brief Queue an i2c write request and block until the request completes, or until timeout until occurs.
 *
 * @param i2c Either \ref i2c0 or \ref i2c1
 * @param addr 7-bit address of device to write to
 * @param src Pointer to data to send
 * @param len Length of data in bytes to send
 * @param nostop  If true, master retains control of the bus at the end of the transfer (no Stop is issued),
 *           and the next transfer will begin with a Restart rather than a Start.
 * @param until The absolute time that the block will wait until the entire transaction is complete. Note, an individual timeout of
 *           this value divided by the length of data is applied for each byte transfer, so if the first or subsequent
 *           bytes fails to transfer within that sub timeout, the function will return with an error.
 *
 * @return Number of bytes read, or PICO_ERROR_GENERIC if an error occurred.
 */
int async_i2c_write_blocking_until(i2c_inst_t *i2c, uint8_t addr, const uint8_t *src, size_t len,
                                   bool nostop, absolute_time_t until);

/**
 * @brief Queue an i2c read request and block until the request completes, or until timeout until occurs.
 *
 * @param i2c Either @ref i2c0 or @ref i2c1
 * @param addr 7-bit address of device to read from
 * @param dst Pointer to buffer to receive data
 * @param len Length of data in bytes to receive
 * @param nostop  If true, master retains control of the bus at the end of the transfer (no Stop is issued),
 *           and the next transfer will begin with a Restart rather than a Start.
 * @param until The absolute time that the block will wait until the entire transaction is complete.
 * @return Number of bytes read, or PICO_ERROR_GENERIC if an error occurred.
 */
int async_i2c_read_blocking_until(i2c_inst_t *i2c, uint8_t addr, uint8_t *dst, size_t len,
                                  bool nostop, absolute_time_t until);

/**
 * @brief Initialize async i2c and the corresponding i2c hardware
 *
 * @attention At least one i2c bus must be initialized
 *
 * @param i2c0_sda The I2C0 SDA pin to assign, or -1 if i2c0 is unused
 * @param i2c0_scl The I2C0 SCL pin to assign, or -1 if i2c0 is unused
 * @param i2c1_sda The I2C1 SDA pin to assign, or -1 if i2c1 is unused
 * @param i2c1_scl The I2C1 SCL pin to assign, or -1 if i2c1 is unused
 * @param baudrate The data rate of the i2c bus in Hz
 * @param bus_timeout_ms The timeout from start of a transaction in ms
 */
void async_i2c_init(int i2c0_sda, int i2c0_scl, int i2c1_sda, int i2c1_scl,
                    unsigned int baudrate, unsigned int bus_timeout_ms);

#endif