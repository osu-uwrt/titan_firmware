#include <stdio.h>
#include "hardware/i2c.h"
#include "hardware/irq.h"
#include "pico/binary_info.h"
#include "pico/stdlib.h"
#include "pico/mutex.h"
#include "pico/time.h"

#include "async_i2c.h"
#include "safety.h"

bool async_i2c_initialized = false;
static uint i2c_bus_timeout;

#define I2C_REQ_QUEUE_SIZE 16

static inline bool i2c_reserved_addr(uint8_t addr) {
    return (addr & 0x78) == 0 || (addr & 0x78) == 0x78;
}

// Define Ring Bufer
struct pending_i2c_request {
    const struct async_i2c_request *pending_request;
    bool *in_progress;
} pending_request_queue[I2C_REQ_QUEUE_SIZE];

static int request_queue_next_entry = 0;
static int request_queue_next_space = 0;

// ========================================
// Bus Management Functions
// ========================================

static struct active_transfer_data {
    /* Request States
     * I2C_IDLE: No data is transmitting. The transfer needs to be started by enqueue to work
     *   - When setting to IDLE care must be taken to disable interrupts to prevent race conditions
     * I2C_PENDING: The bus has been taken over, but the transaction has not been started yet
     *   - Used to reserve the i2c bus during critical sections
     * I2C_TRANSMITTING: Request is currently transmitting tx_buffer
     * I2C_RECEIVING: Request is currently receiving into rx_buffer
     * I2C_DONE: Request is done being processed, but has not released hardware, probably in a callback
     *   - Prevents data from being sent when an interrupt handler is running, possibly stalling the bus
     */
    enum {I2C_IDLE, I2C_PENDING, I2C_TRANSMITTING, I2C_RECEIVING, I2C_DONE} request_state;

    // None of these values are valid when request_state is I2C_IDLE
    const struct async_i2c_request *request;
    bool *in_progress;
    uint16_t bytes_sent;
    uint16_t bytes_received;
    uint16_t receive_commands_queued;
    alarm_id_t timeout_alarm;
    bool alarm_active;
} active_transfer = {.request_state = I2C_IDLE};

#define has_irq_pending(i2c_inst, irq_name) (i2c_inst->hw->raw_intr_stat & I2C_IC_RAW_INTR_STAT_##irq_name##_BITS)

static void async_i2c_start_transmit_stage(void) {
    i2c_inst_t *i2c = active_transfer.request->i2c;
    active_transfer.request_state = I2C_TRANSMITTING;
        
    // Set address
    i2c->hw->enable = 0;
    while (i2c->hw->enable_status & I2C_IC_ENABLE_STATUS_IC_EN_BITS) {
        tight_loop_contents();
    }
    i2c->hw->tar = active_transfer.request->address;
    i2c->hw->enable = 1;

    hard_assert_if(ASYNC_I2C, i2c->hw->rxflr != 0);
    hard_assert_if(ASYNC_I2C, i2c->hw->txflr != 0);

    // Send command
    while (i2c_get_write_available(i2c) && active_transfer.bytes_sent < active_transfer.request->bytes_to_send) {
        bool first = active_transfer.bytes_sent == 0;
        bool last = active_transfer.bytes_sent + 1 == active_transfer.request->bytes_to_send;

        i2c->hw->data_cmd =
                bool_to_bit(first && i2c->restart_on_next) << I2C_IC_DATA_CMD_RESTART_LSB |
                bool_to_bit(last && !active_transfer.request->nostop) << I2C_IC_DATA_CMD_STOP_LSB |
                active_transfer.request->tx_buffer[active_transfer.bytes_sent];

        active_transfer.bytes_sent++;
    }

    if (active_transfer.bytes_sent < active_transfer.request->bytes_to_send) {
        // Enable the tx_empty interrupt if not all of the data is written 
        hw_set_bits(&i2c->hw->intr_mask, I2C_IC_INTR_MASK_M_TX_EMPTY_BITS);
    }
}

static void async_i2c_start_receive_stage(void) {
    i2c_inst_t *i2c = active_transfer.request->i2c;
    active_transfer.request_state = I2C_RECEIVING;

    i2c->hw->enable = 0;
    while (i2c->hw->enable_status & I2C_IC_ENABLE_STATUS_IC_EN_BITS) {
        tight_loop_contents();
    }
    i2c->hw->tar = active_transfer.request->address;
    i2c->hw->enable = 1;

    hard_assert_if(ASYNC_I2C, i2c->hw->rxflr != 0);
    hard_assert_if(ASYNC_I2C, i2c->hw->txflr != 0);
    
    while (i2c_get_write_available(i2c) && active_transfer.receive_commands_queued < active_transfer.request->bytes_to_receive) {
        bool first = active_transfer.receive_commands_queued == 0;
        bool last = active_transfer.receive_commands_queued + 1 == active_transfer.request->bytes_to_receive;

        i2c->hw->data_cmd =
                bool_to_bit(i2c->restart_on_next) << I2C_IC_DATA_CMD_RESTART_LSB |
                bool_to_bit(last && !active_transfer.request->nostop) << I2C_IC_DATA_CMD_STOP_LSB |
                I2C_IC_DATA_CMD_CMD_BITS; // -> 1 for read

        active_transfer.receive_commands_queued++;
    }

    if (active_transfer.receive_commands_queued < active_transfer.request->bytes_to_receive) {
        // Enable the tx_empty interrupt if not all of the data is written 
        hw_set_bits(&i2c->hw->intr_mask, I2C_IC_INTR_MASK_M_RX_FULL_BITS);
    }
}

static int64_t async_i2c_timeout_callback(alarm_id_t id, void *user_data) {
    active_transfer.alarm_active = false;
    i2c_inst_t *i2c = active_transfer.request->i2c;

    hw_set_bits(&i2c->hw->enable, I2C_IC_ENABLE_ABORT_BITS);
    return 0;
}

/**
 * @brief Starts performing request on the bus
 * The caller must ensure that it is the ONLY code in control of the bus
 * This can be done in critical sections of code if the bus is in IDLE
 * 
 * INTERRUPT SAFE* (If the code owns the bus)
 * 
 * @param request Request to start
 * @param in_progress The pointer to the in_progress boolean
 */
int total_allocated_alarm_count = 0;
static void async_i2c_start_request_internal(const struct async_i2c_request *request, bool *in_progress) {
    active_transfer.request = request;
    active_transfer.in_progress = in_progress;
    active_transfer.bytes_received = 0;
    active_transfer.receive_commands_queued = 0;
    active_transfer.bytes_sent = 0;

    valid_params_if(ASYNC_I2C, active_transfer.request_state == I2C_PENDING || active_transfer.request_state == I2C_DONE);
    hard_assert_if(ASYNC_I2C, active_transfer.alarm_active);
    total_allocated_alarm_count++;
    alarm_id_t alarm_id = add_alarm_in_ms(i2c_bus_timeout, &async_i2c_timeout_callback, NULL, true);
    hard_assert_if(ASYNC_I2C, alarm_id < 0);
    active_transfer.timeout_alarm = alarm_id;
    active_transfer.alarm_active = true;

    // TODO: No stop on bus turnaround?

    if (request->bytes_to_send > 0) {
        async_i2c_start_transmit_stage();
    } else if (request->bytes_to_receive > 0) {
        async_i2c_start_receive_stage();
    } else {
        *in_progress = false;
    }
}

/**
 * @brief Common irq handler for i2c related tasks
 * 
 * @param i2c The i2c inst which caused the interrupt
 */
static void async_i2c_common_irq_handler(i2c_inst_t *i2c) {
    hard_assert_if(ASYNC_I2C, i2c != active_transfer.request->i2c);

    // Handle software issues first
    if (has_irq_pending(i2c, TX_OVER)) {
        // Tx Data Buffer Overflow
        // TODO: See if these can be handled as a fault instead of panic
        panic("I2C TX Buffer Overflow");
    }
    if (has_irq_pending(i2c, RX_UNDER)) {
        // Read when no data available
        panic("I2C RX Underflow");
    }
    if (has_irq_pending(i2c, RX_OVER)) {
        // The i2c buffer wasn't read in time
        // This means that the IRQ didn't fire in time and data was lost
        panic("I2C RX Buffer Overflow");
    }

    // Handle error states
    if (has_irq_pending(i2c, TX_ABRT)) {
        if (active_transfer.alarm_active) {
            cancel_alarm(active_transfer.timeout_alarm);
            active_transfer.alarm_active = false;
        }

        // Transmit abort
        uint32_t abort_reason = i2c->hw->tx_abrt_source;
        i2c->hw->clr_tx_abrt;

        hw_clear_bits(&i2c->hw->intr_mask, I2C_IC_INTR_MASK_M_TX_EMPTY_BITS | I2C_IC_INTR_MASK_M_RX_FULL_BITS);
        i2c->restart_on_next = active_transfer.request->nostop;
        active_transfer.request_state = I2C_DONE;

        // These are the tx aborts that are possible failures from the bus rather than internal
        const uint32_t valid_bus_faults = I2C_IC_TX_ABRT_SOURCE_ABRT_USER_ABRT_BITS | 
                                          I2C_IC_TX_ABRT_SOURCE_ABRT_SBYTE_ACKDET_BITS | 
                                          I2C_IC_TX_ABRT_SOURCE_ABRT_HS_ACKDET_BITS | 
                                          I2C_IC_TX_ABRT_SOURCE_ABRT_GCALL_NOACK_BITS | 
                                          I2C_IC_TX_ABRT_SOURCE_ABRT_TXDATA_NOACK_BITS | 
                                          I2C_IC_TX_ABRT_SOURCE_ABRT_7B_ADDR_NOACK_BITS;

        if (abort_reason & ~(I2C_IC_TX_ABRT_SOURCE_TX_FLUSH_CNT_BITS | valid_bus_faults)) {
            printf("Unexpected TX Abort: %d\n", abort_reason);
            safety_raise_fault(FAULT_ASYNC_I2C_ERROR);
        }
        
        *active_transfer.in_progress = false;
        if (active_transfer.request->failed_callback) {
            active_transfer.request->failed_callback(active_transfer.request, abort_reason);
        }
    }

    // Handle normal states
    if (has_irq_pending(i2c, TX_EMPTY)) {
        // Transmit buffer needs to be filled (cleared by hw)

        while (i2c_get_write_available(i2c) && active_transfer.bytes_sent < active_transfer.request->bytes_to_send) {
            bool first = active_transfer.bytes_sent == 0;
            bool last = active_transfer.bytes_sent + 1 == active_transfer.request->bytes_to_send;

            i2c->hw->data_cmd =
                    bool_to_bit(first && i2c->restart_on_next) << I2C_IC_DATA_CMD_RESTART_LSB |
                    bool_to_bit(last && !active_transfer.request->nostop) << I2C_IC_DATA_CMD_STOP_LSB |
                    active_transfer.request->tx_buffer[active_transfer.bytes_sent];

            active_transfer.bytes_sent++;
        }

        if (active_transfer.bytes_sent == active_transfer.request->bytes_to_send) {
            hw_clear_bits(&i2c->hw->intr_mask, I2C_IC_INTR_MASK_M_TX_EMPTY_BITS);
        }
        
    }
    if (has_irq_pending(i2c, RX_FULL)) {
        // Receive buffer needs to be read in (cleared by hw)
        while (i2c_get_read_available(i2c)) {
            assert(active_transfer.bytes_received < active_transfer.request->bytes_to_receive);

            active_transfer.request->rx_buffer[active_transfer.bytes_received] = (uint8_t) i2c->hw->data_cmd;
            active_transfer.bytes_received++;
        }

        while (i2c_get_write_available(i2c) && active_transfer.receive_commands_queued < active_transfer.request->bytes_to_receive) {
            bool first = active_transfer.receive_commands_queued == 0;
            bool last = active_transfer.receive_commands_queued + 1 == active_transfer.request->bytes_to_receive;

            i2c->hw->data_cmd =
                    bool_to_bit(i2c->restart_on_next) << I2C_IC_DATA_CMD_RESTART_LSB |
                    bool_to_bit(last && !active_transfer.request->nostop) << I2C_IC_DATA_CMD_STOP_LSB |
                    I2C_IC_DATA_CMD_CMD_BITS; // -> 1 for read

            active_transfer.receive_commands_queued++;
        }

        if (active_transfer.receive_commands_queued < active_transfer.request->bytes_to_receive) {
            // Enable the tx_empty interrupt if not all of the data is written 
            hw_set_bits(&i2c->hw->intr_mask, I2C_IC_INTR_MASK_M_RX_FULL_BITS);
        }

    }
    if (has_irq_pending(i2c, STOP_DET)) {
        // Cleanup previous request
        i2c->hw->clr_stop_det;
        hw_clear_bits(&i2c->hw->intr_mask, I2C_IC_INTR_MASK_M_TX_EMPTY_BITS | I2C_IC_INTR_MASK_M_RX_FULL_BITS);
        i2c->restart_on_next = active_transfer.request->nostop;

        // Handle any remaining data in the receive buffer after stop
        while (i2c_get_read_available(i2c)) {
            assert(active_transfer.bytes_received < active_transfer.request->bytes_to_receive);

            active_transfer.request->rx_buffer[active_transfer.bytes_received] = (uint8_t) i2c->hw->data_cmd;
            active_transfer.bytes_received++;
        }

        // Only do processing on start bit if the data was properly received
        // If an abort occurs then stop occurs before abort, so it might not be detected for an abort
        if (active_transfer.request_state == I2C_TRANSMITTING && active_transfer.bytes_sent == active_transfer.request->bytes_to_send && active_transfer.request->bytes_to_receive > 0) {
            async_i2c_start_receive_stage();
        } else if (active_transfer.bytes_sent == active_transfer.request->bytes_to_send && active_transfer.bytes_received == active_transfer.request->bytes_to_receive) {
            if (active_transfer.alarm_active) {
                cancel_alarm(active_transfer.timeout_alarm);
                active_transfer.alarm_active = false;
            }

            // Do processing for final bit
            hard_assert_if(ASYNC_I2C, active_transfer.receive_commands_queued != active_transfer.request->bytes_to_receive);

            active_transfer.request_state = I2C_DONE;
            if (active_transfer.request->next_req_on_success) {
                async_i2c_start_request_internal(active_transfer.request->next_req_on_success, active_transfer.in_progress);
            } else {
                *active_transfer.in_progress = false;
            }

            if (active_transfer.request->completed_callback) {
                active_transfer.request->completed_callback(active_transfer.request);
            }
        }
    }

    // Handle transaction complete
    if (active_transfer.request_state == I2C_DONE) {
        if (request_queue_next_entry != request_queue_next_space) {
            // Get next piece of data in the queue
            async_i2c_start_request_internal(pending_request_queue[request_queue_next_entry].pending_request,
                                            pending_request_queue[request_queue_next_entry].in_progress);

            // Increment ring buffer
            request_queue_next_entry = (request_queue_next_entry + 1) % I2C_REQ_QUEUE_SIZE;
        } else {
            active_transfer.request_state = I2C_IDLE;
        }
    }
}

// ========================================
// Queuing Related Functions
// ========================================

/**
 * @brief Validates an async_i2c_request. Validates all chained requests recusrively
 * 
 * @param request The request to validate recursively
 * @return true  Request is okay to send
 * @return false Part of the request has an invalid value
 */
static bool async_i2c_validate_request(const struct async_i2c_request *request) {
    // Check for reserved i2c addresses
    if (request->address >= 0x80 || i2c_reserved_addr(request->address)) {
        return false;
    }

    bool has_transfer = false;
    if (request->bytes_to_send > 0) {
        if (request->tx_buffer != NULL) {
            has_transfer = true;
        } else {
            return false;
        }
    }

    if (request->bytes_to_receive > 0) {
        if (request->rx_buffer != NULL) {
            has_transfer = true;
        } else {
            return false;
        }
    }

    // Don't allow requests where no data is transferred
    if (!has_transfer) {
        return false;
    }

    // Recursively check sub-requests
    if (request->next_req_on_success) {
        if (!async_i2c_validate_request(request->next_req_on_success)) {
            return false;
        }
    }

    return true;
}

/**
 * @brief Returns if there is space in the async i2c queue
 * 
 * INITIALIZATION REQUIRED
 * INTERRUPT SAFE* (This might return true but if another command queues after queue still might not be safe to call)
 * 
 * @return true  Space is available in the queue
 * @return false No space is available in the queue for new requests
 */
static bool async_i2c_queue_full(void) {
    return ((request_queue_next_space + 1) % I2C_REQ_QUEUE_SIZE) == request_queue_next_entry;
}

void async_i2c_enqueue(const struct async_i2c_request *request, bool *in_progress) {
    if (!async_i2c_initialized) {
        panic("Aync I2C not initialized");
    }
    
    valid_params_if(ASYNC_I2C, async_i2c_validate_request(request));
    invalid_params_if(ASYNC_I2C, *in_progress);

    // Disable interrupts during operation to prevent corruption
    uint32_t prev_interrupt = save_and_disable_interrupts();

    if (async_i2c_queue_full()) {
        restore_interrupts(prev_interrupt);
        printf("Buffer overflow enqueuing i2c commands\n");
        safety_raise_fault(FAULT_ASYNC_I2C_ERROR);
        return;
    }
    
    *in_progress = true;

    bool can_transfer_immediately = active_transfer.request_state == I2C_IDLE;
    if (can_transfer_immediately) {
        // Don't queue if the bus is idle, just send it
        // Reserve bus to prevent any other calls to this function reserving the bus as well
        active_transfer.request_state = I2C_PENDING;
    } else {
        // Queue in data
        pending_request_queue[request_queue_next_space].pending_request = request;
        pending_request_queue[request_queue_next_space].in_progress = in_progress;

        // Increment ring buffer
        request_queue_next_space = (request_queue_next_space + 1) % I2C_REQ_QUEUE_SIZE;
    }

    restore_interrupts(prev_interrupt);

    if (can_transfer_immediately) {
        async_i2c_start_request_internal(request, in_progress);
    }
}

void async_i2c0_irq_handler(void) {
    async_i2c_common_irq_handler(i2c0);
}

void async_i2c1_irq_handler(void) {
    async_i2c_common_irq_handler(i2c1);
}

static void async_i2c_configure_interrupt_hw(i2c_inst_t *i2c) {
    i2c->hw->intr_mask = I2C_IC_INTR_MASK_M_STOP_DET_BITS | I2C_IC_INTR_MASK_M_TX_ABRT_BITS | 
                          I2C_IC_INTR_MASK_M_TX_OVER_BITS | I2C_IC_INTR_MASK_M_RX_OVER_BITS |
                          I2C_IC_INTR_MASK_M_RX_UNDER_BITS;

    i2c->hw->rx_tl = 10;
    i2c->hw->tx_tl = 6;
}

void async_i2c_init(uint baudrate, uint bus_timeout_ms) {
    i2c_bus_timeout = bus_timeout_ms;

    i2c_init(SENSOR_I2C_HW, baudrate);
    gpio_set_function(SENSOR_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(SENSOR_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(SENSOR_SDA_PIN);
    gpio_pull_up(SENSOR_SCL_PIN);
    irq_set_exclusive_handler(I2C0_IRQ, &async_i2c0_irq_handler);
    async_i2c_configure_interrupt_hw(SENSOR_I2C_HW);

    i2c_init(BOARD_I2C_HW, baudrate);
    gpio_set_function(BOARD_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(BOARD_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(BOARD_SDA_PIN);
    gpio_pull_up(BOARD_SCL_PIN);
    irq_set_exclusive_handler(I2C1_IRQ, &async_i2c1_irq_handler);
    async_i2c_configure_interrupt_hw(BOARD_I2C_HW);

    irq_set_enabled(I2C0_IRQ, true);
    irq_set_enabled(I2C1_IRQ, true);

    // Make the I2C pins available to picotool
    bi_decl_if_func_used(bi_2pins_with_func(SENSOR_SDA_PIN, SENSOR_SCL_PIN, GPIO_FUNC_I2C));
    bi_decl_if_func_used(bi_2pins_with_func(BOARD_SDA_PIN, BOARD_SCL_PIN, GPIO_FUNC_I2C));

    async_i2c_initialized = true;
}