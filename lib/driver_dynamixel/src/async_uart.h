#ifndef DYNAMIXEL__ASYNC_UART_H
#define DYNAMIXEL__ASYNC_UART_H

#include "hardware/pio.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

enum async_uart_tx_err {
    /**
     * @brief Successfully transmitted data
     */
    ASYNC_UART_TX_OK = 0,

    /**
     * @brief Could not transmit as another transmit was in progress
     */
    ASYNC_UART_TX_BUSY,
};

enum async_uart_rx_err {
    /**
     * @brief Receive successful (all requested bytes received)
     */
    ASYNC_UART_RX_OK = 0,
    /**
     * @brief Could not receive as another receive was in progress
     */
    ASYNC_UART_RX_BUSY,
    /**
     * @brief No data received within timeout
     */
    ASYNC_UART_RX_TIMEOUT,
    /**
     * @brief Not all of the requested data was received within timeout
     */
    ASYNC_UART_RX_INCOMPLETE,
    /**
     * @brief A framing error was detected during receive
     */
    ASYNC_UART_RX_FRAME_ERROR,

    /**
     * @brief Data was lost as the internal FIFO overflowed
     * (async_uart_read was not called fast enough)
     *
     * @note This error condition may be raised on a transfer from FIFO overflows from bytes before/after the transfer
     * However, a write with preserve_read set to false will clear this fault
     * This is due the possibility of data loss between the interrupt latency of processing the previous request, the
     * FIFO may overflow before a read can be processed. However any latencies which cause this edge case can also
     * cause actual data loss from the DMA not being serviced in time when chaining transfers.
     */
    ASYNC_UART_RX_DATA_LOST,

    /**
     * @brief The receive was aborted as a transmit was requested while the read was still in progress
     */
    ASYNC_UART_RX_ABORTED,
};

/**
 * @brief Callback on successful write/write error
 */
typedef void (*async_uart_on_write)(enum async_uart_tx_err error);

/**
 * @brief Callback on successful read/read error
 *
 * @param error The error code
 * @param data Pointer to buffer data was recieved into (only valid if error = ASYNC_UART_RX_OK)
 * @param len The length of received data (only valid if error = ASYNC_UART_RX_OK)
 */
typedef void (*async_uart_on_read)(enum async_uart_rx_err error, uint8_t *data, size_t len);

/**
 * @brief Initialize async multidrop uart
 *
 * @param pio The PIO block to use
 * @param sm The PIO sm to use
 * @param pin The pin to use
 * @param baud The baud rate to use for transactions
 * @param timeout_ms The maximum time between received bytes before a receive timeout occurs
 */
void async_uart_init(PIO pio, unsigned int sm, unsigned int pin, unsigned int baud, unsigned int timeout_ms);

/**
 * @brief Transmit the requested data over uart multidrop
 * Note this can also be used to reset the read state if a length of 0 is written
 *
 * @param data The data to transmit (can be NULL if len is 0)
 * @param len Length of data to transmit (can be 0)
 * @param preserve_read Setting to true will not clear the read buffer/error state before transmitting
 * @param cb The callback to fire after write/on error (can be NULL)
 */
void async_uart_write(const uint8_t *data, size_t len, bool preserve_read, async_uart_on_write cb);

/**
 * @brief Receive the requested data over uart multidrop
 * NOTE: The internal FIFO is only 4 bytes deep. Any delays when calling this function can result in data loss
 * At 57600 baud, this function must be called within ~690us if data is actively being transmitted before data is lost
 *
 * @param data The buffer to receive data into
 * @param len The length of data to receive
 * @param cb The callback to fire upon successful receive/error
 */
void async_uart_read(uint8_t *data, size_t len, async_uart_on_read cb);

#endif
