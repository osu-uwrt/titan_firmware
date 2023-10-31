#include "driver/async_i2c.h"

#include "hardware/gpio.h"
#include "hardware/i2c.h"
#include "hardware/irq.h"
#include "hardware/sync.h"
#include "pico/assert.h"
#include "pico/time.h"

#include <stdio.h>

// Note that this code can be used with titan_logger if you're tracing down i2c gremlins
// Best of luck if you need to use this, and be aware printfs in interrupts can bite you

// #include "titan/logger.h"
// #undef LOGGING_UNIT_NAME
// #define LOGGING_UNIT_NAME "async_i2c"

#define LOG_DEBUG(...)                                                                                                 \
    do {                                                                                                               \
    } while (0)

bool async_i2c_initialized = false;
static uint i2c_bus_timeout;
static bool i2c0_initialized = false;
static bool i2c1_initialized = false;

static inline bool i2c_reserved_addr(uint8_t addr) {
    return (addr & 0x78) == 0 || (addr & 0x78) == 0x78;
}

// Define Ring Bufer
struct pending_i2c_request {
    const struct async_i2c_request *pending_request;
    volatile bool *in_progress;
} pending_request_queue[I2C_REQ_QUEUE_SIZE];

static int request_queue_next_entry = 0;
static int request_queue_next_space = 0;

#define i2c_num_to_inst(i2c_num) (i2c_num == 1 ? i2c1 : i2c0)

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
    enum { I2C_IDLE, I2C_PENDING, I2C_TRANSMITTING, I2C_RECEIVING, I2C_DONE } request_state;

    // None of these values are valid when request_state is I2C_IDLE
    const struct async_i2c_request *request;
    volatile bool *in_progress;
    uint16_t bytes_sent;
    uint16_t bytes_received;
    uint16_t receive_commands_queued;
    alarm_id_t timeout_alarm;
    bool alarm_active;
} active_transfer = { .request_state = I2C_IDLE };

#define has_irq_pending(i2c_inst, irq_name) (i2c_inst->hw->raw_intr_stat & I2C_IC_RAW_INTR_STAT_##irq_name##_BITS)

static void async_i2c_start_transmit_stage(void) {
    i2c_inst_t *i2c = i2c_num_to_inst(active_transfer.request->i2c_num);
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

        i2c->hw->data_cmd = bool_to_bit(first && i2c->restart_on_next) << I2C_IC_DATA_CMD_RESTART_LSB |
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
    i2c_inst_t *i2c = i2c_num_to_inst(active_transfer.request->i2c_num);
    active_transfer.request_state = I2C_RECEIVING;

    i2c->hw->enable = 0;
    while (i2c->hw->enable_status & I2C_IC_ENABLE_STATUS_IC_EN_BITS) {
        tight_loop_contents();
    }
    i2c->hw->tar = active_transfer.request->address;
    i2c->hw->enable = 1;

    hard_assert_if(ASYNC_I2C, i2c->hw->rxflr != 0);
    hard_assert_if(ASYNC_I2C, i2c->hw->txflr != 0);

    while (i2c_get_write_available(i2c) &&
           active_transfer.receive_commands_queued < active_transfer.request->bytes_to_receive) {
        bool first = active_transfer.receive_commands_queued == 0;
        bool last = active_transfer.receive_commands_queued + 1 == active_transfer.request->bytes_to_receive;

        i2c->hw->data_cmd = bool_to_bit(first && i2c->restart_on_next) << I2C_IC_DATA_CMD_RESTART_LSB |
                            bool_to_bit(last && !active_transfer.request->nostop) << I2C_IC_DATA_CMD_STOP_LSB |
                            I2C_IC_DATA_CMD_CMD_BITS;  // -> 1 for read

        active_transfer.receive_commands_queued++;
    }

    if (active_transfer.receive_commands_queued < active_transfer.request->bytes_to_receive) {
        // Enable the tx_empty interrupt if not all of the data is written
        hw_set_bits(&i2c->hw->intr_mask, I2C_IC_INTR_MASK_M_RX_FULL_BITS);
    }
}

static int64_t async_i2c_timeout_callback(__unused alarm_id_t id, __unused void *user_data) {
    active_transfer.alarm_active = false;
    i2c_inst_t *i2c = i2c_num_to_inst(active_transfer.request->i2c_num);

    LOG_DEBUG("Timeout Called (0x%p)", active_transfer.request);
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
static bool async_i2c_start_request_internal(const struct async_i2c_request *request, volatile bool *in_progress) {
    LOG_DEBUG("Starting request 0x%p", request);
    active_transfer.request = request;
    active_transfer.in_progress = in_progress;
    active_transfer.bytes_received = 0;
    active_transfer.receive_commands_queued = 0;
    active_transfer.bytes_sent = 0;

    valid_params_if(ASYNC_I2C,
                    active_transfer.request_state == I2C_PENDING || active_transfer.request_state == I2C_DONE);
    hard_assert_if(ASYNC_I2C, active_transfer.alarm_active);
    total_allocated_alarm_count++;
    alarm_id_t alarm_id;
    if (is_nil_time(request->timeout)) {
        alarm_id = add_alarm_in_ms(i2c_bus_timeout, &async_i2c_timeout_callback, NULL, false);
    }
    else {
        alarm_id = add_alarm_at(request->timeout, &async_i2c_timeout_callback, NULL, false);

        if (alarm_id == 0 && time_reached(request->timeout)) {
            if (request->failed_callback) {
                // Callback failed that the request was aborted
                request->failed_callback(request, I2C_IC_TX_ABRT_SOURCE_ABRT_USER_ABRT_BITS);
            }
            *in_progress = false;
            return false;
        }
    }
    hard_assert(alarm_id >= 0);
    active_transfer.timeout_alarm = alarm_id;
    active_transfer.alarm_active = true;

    if (request->bytes_to_send > 0) {
        async_i2c_start_transmit_stage();
    }
    else if (request->bytes_to_receive > 0) {
        async_i2c_start_receive_stage();
    }
    else {
        *in_progress = false;
        return false;
    }

    return true;
}

/**
 * @brief Common irq handler for i2c related tasks
 *
 * @param i2c The i2c inst which caused the interrupt
 */
static void async_i2c_common_irq_handler(i2c_inst_t *i2c) {
    if (!(i2c->hw->raw_intr_stat & i2c->hw->intr_mask)) {
        return;  // In the event the IRQ went away, ignore it
        // Could happen if the full/empty irq tripped during fill/emptying
    }
    LOG_DEBUG("Interrupt callback on %s for 0x%p, active interrupts 0x%x",
              (i2c == i2c0 ? "i2c0" : (i2c == i2c1 ? "i2c1" : "Unknown")), active_transfer.request,
              i2c->hw->raw_intr_stat & i2c->hw->intr_mask);

    hard_assert_if(ASYNC_I2C, i2c_hw_index(i2c) != active_transfer.request->i2c_num);

    // Handle software issues first
    if (has_irq_pending(i2c, TX_OVER)) {
        // Tx Data Buffer Overflow
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
    bool transfer_aborted = false;
    if (has_irq_pending(i2c, TX_ABRT)) {
        transfer_aborted = true;

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

        *active_transfer.in_progress = false;
        if (active_transfer.request->failed_callback) {
            active_transfer.request->failed_callback(active_transfer.request, abort_reason);
        }
    }

    // Handle normal states
    if (!transfer_aborted && has_irq_pending(i2c, TX_EMPTY)) {
        // Transmit buffer needs to be filled (cleared by hw)

        while (i2c_get_write_available(i2c) && active_transfer.bytes_sent < active_transfer.request->bytes_to_send) {
            bool first = active_transfer.bytes_sent == 0;
            bool last = active_transfer.bytes_sent + 1 == active_transfer.request->bytes_to_send;

            i2c->hw->data_cmd = bool_to_bit(first && i2c->restart_on_next) << I2C_IC_DATA_CMD_RESTART_LSB |
                                bool_to_bit(last && !active_transfer.request->nostop) << I2C_IC_DATA_CMD_STOP_LSB |
                                active_transfer.request->tx_buffer[active_transfer.bytes_sent];

            active_transfer.bytes_sent++;
        }

        if (active_transfer.bytes_sent == active_transfer.request->bytes_to_send) {
            hw_clear_bits(&i2c->hw->intr_mask, I2C_IC_INTR_MASK_M_TX_EMPTY_BITS);
        }
    }
    if (!transfer_aborted && has_irq_pending(i2c, RX_FULL)) {
        // Receive buffer needs to be read in (cleared by hw)
        while (i2c_get_read_available(i2c)) {
            assert(active_transfer.bytes_received < active_transfer.request->bytes_to_receive);

            active_transfer.request->rx_buffer[active_transfer.bytes_received] = (uint8_t) i2c->hw->data_cmd;
            active_transfer.bytes_received++;
        }

        while (i2c_get_write_available(i2c) &&
               active_transfer.receive_commands_queued < active_transfer.request->bytes_to_receive) {
            bool first = active_transfer.receive_commands_queued == 0;
            bool last = active_transfer.receive_commands_queued + 1 == active_transfer.request->bytes_to_receive;

            i2c->hw->data_cmd = bool_to_bit(first && i2c->restart_on_next) << I2C_IC_DATA_CMD_RESTART_LSB |
                                bool_to_bit(last && !active_transfer.request->nostop) << I2C_IC_DATA_CMD_STOP_LSB |
                                I2C_IC_DATA_CMD_CMD_BITS;  // -> 1 for read

            active_transfer.receive_commands_queued++;
        }

        if (active_transfer.receive_commands_queued < active_transfer.request->bytes_to_receive) {
            // Enable the tx_empty interrupt if not all of the data is written
            hw_set_bits(&i2c->hw->intr_mask, I2C_IC_INTR_MASK_M_RX_FULL_BITS);
        }
    }
    if (has_irq_pending(i2c, STOP_DET)) {
        LOG_DEBUG("Stop Detected");
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
        if (transfer_aborted) {
            LOG_DEBUG("Transfer aborted");
            // Do nothing on aborted transfer
        }
        else if (active_transfer.request_state == I2C_TRANSMITTING &&
                 active_transfer.bytes_sent == active_transfer.request->bytes_to_send &&
                 active_transfer.request->bytes_to_receive > 0) {
            LOG_DEBUG("Starting receive stage");
            async_i2c_start_receive_stage();
        }
        else if (active_transfer.bytes_sent == active_transfer.request->bytes_to_send &&
                 active_transfer.bytes_received == active_transfer.request->bytes_to_receive) {
            LOG_DEBUG("Finalizing Request 0x%p...", active_transfer.request);
            hard_assert_if(ASYNC_I2C, active_transfer.request_state != I2C_TRANSMITTING &&
                                          active_transfer.request_state != I2C_RECEIVING);

            if (active_transfer.alarm_active) {
                cancel_alarm(active_transfer.timeout_alarm);
                active_transfer.alarm_active = false;
            }

            // Do processing for final bit
            hard_assert_if(ASYNC_I2C,
                           active_transfer.receive_commands_queued != active_transfer.request->bytes_to_receive);

            active_transfer.request_state = I2C_DONE;
            if (active_transfer.request->next_req_on_success) {
                async_i2c_start_request_internal(active_transfer.request->next_req_on_success,
                                                 active_transfer.in_progress);
            }
            else {
                *active_transfer.in_progress = false;
            }

            if (active_transfer.request->completed_callback) {
                active_transfer.request->completed_callback(active_transfer.request);
            }
        }
        else {
            LOG_DEBUG("Unexpected STOP");
        }
    }

    // Handle transaction complete
    if (active_transfer.request_state == I2C_DONE) {
        bool queued = false;

        while (request_queue_next_entry != request_queue_next_space) {
            // Get next piece of data in the queue
            queued = async_i2c_start_request_internal(pending_request_queue[request_queue_next_entry].pending_request,
                                                      pending_request_queue[request_queue_next_entry].in_progress);

            // Increment ring buffer
            request_queue_next_entry = (request_queue_next_entry + 1) % I2C_REQ_QUEUE_SIZE;

            if (queued) {
                break;
            }
        }

        if (!queued) {
            LOG_DEBUG("Transferring to idle state (0x%p)", active_transfer.request);
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
__unused static bool async_i2c_validate_request(const struct async_i2c_request *request) {
    // Ensure i2c_num is valid
    if (request->i2c_num != 1 && request->i2c_num != 0) {
        return false;
    }

    // Check for reserved i2c addresses
    if (request->address >= 0x80 || i2c_reserved_addr(request->address)) {
        return false;
    }

    bool has_transfer = false;
    if (request->bytes_to_send > 0) {
        if (request->tx_buffer != NULL) {
            has_transfer = true;
        }
        else {
            return false;
        }
    }

    if (request->bytes_to_receive > 0) {
        if (request->rx_buffer != NULL) {
            has_transfer = true;
        }
        else {
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

bool async_i2c_enqueue(const struct async_i2c_request *request, volatile bool *in_progress) {
    valid_params_if(ASYNC_I2C, async_i2c_validate_request(request));
    hard_assert(!*in_progress);
    hard_assert_if(ASYNC_I2C, !async_i2c_initialized);

    // Disable interrupts during operation to prevent corruption
    uint32_t prev_interrupt = save_and_disable_interrupts();
    __compiler_memory_barrier();

    if (async_i2c_queue_full()) {
        restore_interrupts(prev_interrupt);
        return false;
    }

    *in_progress = true;

    bool can_transfer_immediately = active_transfer.request_state == I2C_IDLE;
    if (can_transfer_immediately) {
        // Don't queue if the bus is idle, just send it
        // Reserve bus to prevent any other calls to this function reserving the bus as well
        active_transfer.request_state = I2C_PENDING;
    }
    else {
        // Queue in data
        pending_request_queue[request_queue_next_space].pending_request = request;
        pending_request_queue[request_queue_next_space].in_progress = in_progress;

        // Increment ring buffer
        request_queue_next_space = (request_queue_next_space + 1) % I2C_REQ_QUEUE_SIZE;
    }

    __compiler_memory_barrier();
    restore_interrupts(prev_interrupt);

    if (can_transfer_immediately) {
        async_i2c_start_request_internal(request, in_progress);
    }

    return true;
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

static void async_i2c_blocking_successful(const struct async_i2c_request *req) {
    // Set the successful bool
    *((bool *) req->user_data) = true;
}

int async_i2c_write_blocking_until(i2c_inst_t *i2c, uint8_t addr, const uint8_t *src, size_t len, bool nostop,
                                   absolute_time_t until) {
    bool successful = false;

    struct async_i2c_request req = { .i2c_num = i2c_hw_index(i2c),
                                     .address = addr,
                                     .nostop = nostop,
                                     .tx_buffer = src,
                                     .rx_buffer = NULL,
                                     .bytes_to_send = len,
                                     .bytes_to_receive = 0,
                                     .completed_callback = &async_i2c_blocking_successful,
                                     .failed_callback = NULL,
                                     .next_req_on_success = NULL,
                                     .user_data = &successful,
                                     .timeout = until };

    volatile bool in_progress = false;
    async_i2c_enqueue(&req, &in_progress);

    while (in_progress) {
        tight_loop_contents();
    }

    return (successful ? (int) len : PICO_ERROR_GENERIC);
}

int async_i2c_read_blocking_until(i2c_inst_t *i2c, uint8_t addr, uint8_t *dst, size_t len, bool nostop,
                                  absolute_time_t until) {
    bool successful = false;

    struct async_i2c_request req = { .i2c_num = i2c_hw_index(i2c),
                                     .address = addr,
                                     .nostop = nostop,
                                     .tx_buffer = NULL,
                                     .rx_buffer = dst,
                                     .bytes_to_send = 0,
                                     .bytes_to_receive = len,
                                     .completed_callback = &async_i2c_blocking_successful,
                                     .failed_callback = NULL,
                                     .next_req_on_success = NULL,
                                     .user_data = &successful,
                                     .timeout = until };

    volatile bool in_progress = false;
    async_i2c_enqueue(&req, &in_progress);

    while (in_progress) {
        tight_loop_contents();
    }

    return (successful ? (int) len : PICO_ERROR_GENERIC);
}

void async_i2c_init(int i2c0_sda, int i2c0_scl, int i2c1_sda, int i2c1_scl, unsigned int baudrate,
                    unsigned int bus_timeout_ms) {
    hard_assert_if(ASYNC_I2C, async_i2c_initialized);

    LOG_DEBUG("Initializing Async I2C");
    i2c_bus_timeout = bus_timeout_ms;
    i2c0_initialized = false;
    i2c1_initialized = false;

    if (i2c0_sda != -1 && i2c0_scl != -1) {
        // Make sure the pins actually correspond to their corresponding value
        valid_params_if(ASYNC_I2C, (i2c0_sda % 4) == 0 && (i2c0_scl % 4) == 1);

        i2c_init(i2c0, baudrate);
        gpio_set_function(i2c0_sda, GPIO_FUNC_I2C);
        gpio_set_function(i2c0_scl, GPIO_FUNC_I2C);
        gpio_pull_up(i2c0_sda);
        gpio_pull_up(i2c0_scl);
        irq_set_exclusive_handler(I2C0_IRQ, &async_i2c0_irq_handler);
        async_i2c_configure_interrupt_hw(i2c0);

        i2c0_initialized = true;
    }
    else {
        // Invalid parameter if one pin is defined but one is unassigned
        invalid_params_if(ASYNC_I2C, i2c0_sda != -1 || i2c0_scl != -1);
    }

    if (i2c1_sda != -1 && i2c1_scl != -1) {
        // Make sure the pins actually correspond to their corresponding value
        valid_params_if(ASYNC_I2C, (i2c1_sda % 4) == 2 && (i2c1_scl % 4) == 3);

        i2c_init(i2c1, baudrate);
        gpio_set_function(i2c1_sda, GPIO_FUNC_I2C);
        gpio_set_function(i2c1_scl, GPIO_FUNC_I2C);
        gpio_pull_up(i2c1_sda);
        gpio_pull_up(i2c1_scl);
        irq_set_exclusive_handler(I2C1_IRQ, &async_i2c1_irq_handler);
        async_i2c_configure_interrupt_hw(i2c1);

        i2c1_initialized = true;
    }
    else {
        // Invalid parameter if one pin is defined but one is unassigned
        invalid_params_if(ASYNC_I2C, i2c1_sda != -1 || i2c1_scl != -1);
    }

    invalid_params_if(ASYNC_I2C, !i2c0_initialized && !i2c1_initialized);

    irq_set_enabled(I2C0_IRQ, i2c0_initialized);
    irq_set_enabled(I2C1_IRQ, i2c1_initialized);

    async_i2c_initialized = true;
}
