#include <stdio.h>
#include <string.h>
#include "hardware/i2c.h"
#include "hardware/irq.h"
#include "pico/binary_info.h"
#include "pico/stdlib.h"
#include "pico/mutex.h"
#include "pico/time.h"

#include "basic_logger/logging.h"
#undef LOGGING_UNIT_NAME
#define LOGGING_UNIT_NAME "async_i2c_target"
#undef LOGGING_UNIT_LOCAL_LEVEL
#define LOGGING_UNIT_LOCAL_LEVEL LEVEL_INFO

#include "async_i2c_target.h"
#include "safety.h"

bool async_i2c_target_initialized = false;

static inline bool i2c_reserved_addr(uint8_t addr) {
    return (addr & 0x78) == 0 || (addr & 0x78) == 0x78;
}

// I2C Definitions
i2c_inst_t * const i2c_inst = BUILTIN_I2C_HW;
const uint sda_pin = BUILTIN_SDA_PIN;
const uint scl_pin = BUILTIN_SCL_PIN;

// ========================================
// Bus Management Functions
// ========================================

static struct active_transfer_data {
    /* Request States
     * I2C_TARGET_IDLE: The target is idle and processing any active requests on the bus
     * I2C_TARGET_RECEIVING: The target is in the process of receiving a command on the bus
     * I2C_TARGET_CMD_RECEIVED: The command has been succesfully received and is waiting to be read by other code
     * I2C_TARGET_CMD_PROCESSING: The command has been read by code and is waiting for a response to be posted
     * I2C_TARGET_WAITING_FOR_RESPONSE: A response has been requested by the i2c controller but has no data. The callback must then start the response
     * I2C_TARGET_RESPONDING: A response has been posted and is current waiting to be read by the i2c controller
     */
    enum {I2C_TARGET_IDLE, I2C_TARGET_RECEIVING, I2C_TARGET_CMD_RECEIVED, I2C_TARGET_CMD_PROCESSING, I2C_TARGET_WAITING_FOR_RESPONSE, I2C_TARGET_RESPONDING} state;

    // Field valid when in mode I2C_TARGET_RECEIVING and I2C_TARGET_CMD_RECEIVED
    actuator_i2c_cmd_t received_command;
    uint16_t bytes_received;
    uint16_t recv_size;

    // Field valid when in mode I2C_TARGET_RESPONDING
    actuator_i2c_response_t *response;
    uint16_t response_size;
    uint16_t bytes_sent;
} active_transfer = {.state = I2C_TARGET_IDLE};

#define has_irq_pending(i2c_inst, irq_name) (i2c_inst->hw->raw_intr_stat & I2C_IC_RAW_INTR_STAT_##irq_name##_BITS)

/**
 * @brief Fills the transmit queue of the i2c with data in the active_transfer
 */
static void async_i2c_fill_transmit_queue(void) {
    LOG_DEBUG("Filling up I2C queue");
    
    uint8_t *raw_response = (uint8_t*)active_transfer.response;
    while (i2c_get_write_available(i2c_inst) && active_transfer.bytes_sent < active_transfer.response_size){
        i2c_get_hw(i2c_inst)->data_cmd = raw_response[active_transfer.bytes_sent];
        active_transfer.bytes_sent++;
    }

    if (active_transfer.bytes_sent == active_transfer.response_size) {
        hw_clear_bits(&i2c_inst->hw->intr_mask, I2C_IC_INTR_MASK_M_TX_EMPTY_BITS);
    } else {
        hw_set_bits(&i2c_inst->hw->intr_mask, I2C_IC_INTR_MASK_M_TX_EMPTY_BITS);
    }
}

static void async_i2c_restart_hardware(i2c_inst_t *i2c) {
    if (i2c->hw->status & I2C_IC_STATUS_SLV_ACTIVITY_BITS) {
        LOG_ERROR("Cannot restart hw with active read request");
        safety_raise_fault(FAULT_I2C_ERROR);
        return;
    }

    hw_clear_bits(&i2c->hw->enable, I2C_IC_ENABLE_ENABLE_BITS);
    do {tight_loop_contents();} while(i2c->hw->enable_status & I2C_IC_ENABLE_STATUS_IC_EN_BITS);

    hw_set_bits(&i2c->hw->enable, I2C_IC_ENABLE_ENABLE_BITS);
    do {tight_loop_contents();} while(!(i2c->hw->enable_status & I2C_IC_ENABLE_STATUS_IC_EN_BITS));
}

static void async_i2c_target_abort(i2c_inst_t *i2c) {
    active_transfer.state = I2C_TARGET_IDLE;
    async_i2c_restart_hardware(i2c);
}

#define I2C_PROTOCOL_ERR(...) LOG_WARN(__VA_ARGS__); safety_raise_fault(FAULT_I2C_PROTO_ERROR);

/**
 * @brief Common irq handler for i2c related tasks
 * 
 * @param i2c The i2c inst which caused the interrupt
 */
static void async_i2c_common_irq_handler(i2c_inst_t *i2c) {
    LOG_DEBUG("Interrupt callback on %s state %d, active interrupts 0x%x", (i2c == i2c0 ? "i2c0" : (i2c == i2c1 ? "i2c1" : "Unknown")), active_transfer.state, i2c->hw->raw_intr_stat & i2c->hw->intr_mask);

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

    if (has_irq_pending(i2c, TX_ABRT)) {
        LOG_DEBUG("TX_ABRT INT");
        if (active_transfer.state != I2C_TARGET_RESPONDING){
            LOG_ERROR("TX Abort interrupt in invalid state %d", active_transfer.state);
            safety_raise_fault(FAULT_I2C_ERROR);
        }

        // Transmit abort
        uint32_t abort_reason = i2c->hw->tx_abrt_source;
        i2c->hw->clr_tx_abrt;

        I2C_PROTOCOL_ERR("Transmit aborted: %d bytes lost, reason: 0x%x", 
                    (abort_reason>>I2C_IC_TX_ABRT_SOURCE_TX_FLUSH_CNT_LSB) + (active_transfer.response_size-active_transfer.bytes_sent), 
                    abort_reason&(~I2C_IC_TX_ABRT_SOURCE_TX_FLUSH_CNT_BITS));
    }

    // Handle normal states
    if (has_irq_pending(i2c, RX_DONE)) {
        // Transmit done, clean up active transfer
        LOG_DEBUG("RX_DONE INT");
        i2c->hw->clr_rx_done;
        
        if (active_transfer.state == I2C_TARGET_RESPONDING) {
            if (active_transfer.bytes_sent != active_transfer.response_size && i2c_get_hw(i2c)->txflr != 0) {
                I2C_PROTOCOL_ERR("Response terminated early, %d/%d bytes queued, %d in buffer", active_transfer.bytes_sent, active_transfer.response_size, i2c_get_hw(i2c)->txflr);
            }

            active_transfer.state = I2C_TARGET_IDLE;
            hw_clear_bits(&i2c->hw->intr_mask, I2C_IC_INTR_MASK_M_TX_EMPTY_BITS);
        } else {
            I2C_PROTOCOL_ERR("Unexpected rx done in invalid state %d", active_transfer.state);
        }
    }

    if (has_irq_pending(i2c, TX_EMPTY) && (i2c_get_hw(i2c)->intr_mask & I2C_IC_INTR_MASK_M_TX_EMPTY_BITS)) {
        // Transmit buffer needs to be filled (cleared by hw)
        
        if (active_transfer.state != I2C_TARGET_RESPONDING) {
            LOG_ERROR("TX Empty interrupt in invalid state %d", active_transfer.state);
            safety_raise_fault(FAULT_I2C_ERROR);
            hw_clear_bits(&i2c->hw->intr_mask, I2C_IC_INTR_MASK_M_TX_EMPTY_BITS);
        } else {
            async_i2c_fill_transmit_queue();
        }   
    }

    if (has_irq_pending(i2c, RD_REQ)) {
        // Read requested from device
        LOG_DEBUG("RD_REQ INT");
        i2c->hw->clr_rd_req;

        if (active_transfer.state == I2C_TARGET_CMD_PROCESSING || active_transfer.state == I2C_TARGET_CMD_RECEIVED) {
            // Mark as waiting for response so when one is provided it gets immediately sent
            active_transfer.state = I2C_TARGET_WAITING_FOR_RESPONSE;
        } else if (active_transfer.state == I2C_TARGET_RESPONDING) {
            if (active_transfer.bytes_sent < active_transfer.response_size) {
                async_i2c_fill_transmit_queue();
            } else {
                i2c->hw->data_cmd = 0xFF;
                I2C_PROTOCOL_ERR("Too many bytes read");
            }
        } else {
            i2c->hw->data_cmd = 0xFF;
            I2C_PROTOCOL_ERR("Unexpected read request");
        }
    }

    if (has_irq_pending(i2c, RX_FULL)) {
        // Data in receive buffer (cleared by hw)
        LOG_DEBUG("RX_FULL INT");
        
        if (active_transfer.state == I2C_TARGET_IDLE || active_transfer.state == I2C_TARGET_RECEIVING) {
            uint8_t *raw_recv_buffer = (uint8_t*)(&active_transfer.received_command);
            size_t max_recv_buffer_size = sizeof(active_transfer.received_command);

            // Receive buffer needs to be read in (cleared by hw)
            int dropped_bytes = 0;
            while (i2c_get_read_available(i2c)) {
                uint32_t raw_data_cmd = i2c->hw->data_cmd;

                bool can_process_data = true;
                if (raw_data_cmd & I2C_IC_DATA_CMD_FIRST_DATA_BYTE_BITS) {
                    if (active_transfer.state != I2C_TARGET_IDLE){
                        I2C_PROTOCOL_ERR("New command received before previous command finished... dropping previous command");
                    }
                    active_transfer.bytes_received = 0;
                    active_transfer.recv_size = offsetof(actuator_i2c_cmd_t, cmd_id) + 1;
                    active_transfer.state = I2C_TARGET_RECEIVING;
                } else if (active_transfer.state == I2C_TARGET_IDLE){
                    // If in idle state wait until start bit before reading data
                    can_process_data = false;
                } else {
                    can_process_data = (active_transfer.bytes_received < active_transfer.recv_size);
                }

                if (can_process_data) {
                    uint8_t data_byte = (uint8_t) raw_data_cmd;

                    if (active_transfer.bytes_received == offsetof(actuator_i2c_cmd_t, cmd_id)) {
                        active_transfer.recv_size = ACTUATOR_GET_CMD_SIZE(data_byte);
                    }
                    if (active_transfer.recv_size == 0){
                        async_i2c_target_abort(i2c);
                        I2C_PROTOCOL_ERR("Invalid command received %d", data_byte);
                    } else {
                        assert(active_transfer.bytes_received < max_recv_buffer_size);
                        raw_recv_buffer[active_transfer.bytes_received++] = data_byte;
                    }
                } else {
                    dropped_bytes++;
                }
            }

            if (dropped_bytes > 0){
                async_i2c_target_abort(i2c);
                I2C_PROTOCOL_ERR("Dropping %d unexpected bytes", dropped_bytes);
            }

            if (active_transfer.state == I2C_TARGET_RECEIVING && active_transfer.bytes_received == active_transfer.recv_size) {
                uint8_t calculated_crc = actuator_i2c_crc8_calc_command(&active_transfer.received_command, active_transfer.bytes_received);
                if (calculated_crc == active_transfer.received_command.crc8) {
                    LOG_DEBUG("Command received");
                    active_transfer.state = I2C_TARGET_CMD_RECEIVED;
                } else {
                    async_i2c_target_abort(i2c);
                    I2C_PROTOCOL_ERR("Invalid CRC on message, 0x%02x received, 0x%02x calculated", active_transfer.received_command.crc8, calculated_crc);
                }
            }
        } else {
            async_i2c_restart_hardware(i2c);
            I2C_PROTOCOL_ERR("Data sent to device while busy with request, dropping data");
        }
    }
}

void async_i2c0_irq_handler(void) {
    async_i2c_common_irq_handler(i2c0);
}

void async_i2c1_irq_handler(void) {
    async_i2c_common_irq_handler(i2c1);
}

static void async_i2c_configure_interrupt_hw(i2c_inst_t *i2c) {
    i2c->hw->intr_mask =  I2C_IC_INTR_MASK_M_RX_FULL_BITS | I2C_IC_INTR_MASK_M_TX_ABRT_BITS | 
                          I2C_IC_INTR_MASK_M_TX_OVER_BITS | I2C_IC_INTR_MASK_M_RX_OVER_BITS |
                          I2C_IC_INTR_MASK_M_RX_UNDER_BITS | I2C_IC_INTR_MASK_M_RD_REQ_BITS |
                          I2C_IC_INTR_MASK_M_RX_DONE_BITS;

    i2c->hw->rx_tl = 0;
    i2c->hw->tx_tl = 6;
}

// ========================================
// Public Functions
// ========================================

bool async_i2c_target_get_next_command(actuator_i2c_cmd_t *cmd){
    hard_assert_if(ASYNC_I2C_TARGET, active_transfer.state == I2C_TARGET_CMD_PROCESSING);

    if (active_transfer.state == I2C_TARGET_CMD_RECEIVED || active_transfer.state == I2C_TARGET_WAITING_FOR_RESPONSE) {
        memcpy(cmd, &active_transfer.received_command, active_transfer.bytes_received);

        if (active_transfer.state != I2C_TARGET_WAITING_FOR_RESPONSE)
            active_transfer.state = I2C_TARGET_CMD_PROCESSING;

        return true;
    }

    return false;
}

void async_i2c_target_finish_command(actuator_i2c_response_t *response, size_t size){
    hard_assert_if(ASYNC_I2C_TARGET, active_transfer.state != I2C_TARGET_CMD_PROCESSING && active_transfer.state != I2C_TARGET_WAITING_FOR_RESPONSE);
    hard_assert(size <= 65535);

    if (response == NULL || size == 0) {
        if (active_transfer.state == I2C_TARGET_WAITING_FOR_RESPONSE) {
            I2C_PROTOCOL_ERR("I2C attempting to read response with no response to active command");
            async_i2c_target_abort(i2c_inst);
            i2c_inst->hw->data_cmd = 0xFF;  // Send a command since if waiting for a response the read req has been cleared
        } else {
            active_transfer.state = I2C_TARGET_IDLE;
        }
        return;
    }

    LOG_DEBUG("Responding with data");
    
    response->crc8 = actuator_i2c_crc8_calc_response(response, size);
    
    active_transfer.response = response;
    active_transfer.response_size = size;
    active_transfer.bytes_sent = 0;

    if (active_transfer.state == I2C_TARGET_WAITING_FOR_RESPONSE) {
        async_i2c_fill_transmit_queue();
    }
    
    active_transfer.state = I2C_TARGET_RESPONDING;
}

void async_i2c_target_init(uint baudrate, uint8_t i2c_address) {
    invalid_params_if(ASYNC_I2C_TARGET, i2c_reserved_addr(i2c_address));
    LOG_DEBUG("Initializing I2C target with address 0x%02x", i2c_address);

    const uint irq_num = (i2c_inst == i2c0 ? I2C0_IRQ : I2C1_IRQ);
    const irq_handler_t irq_handler = (i2c_inst == i2c0 ? &async_i2c0_irq_handler : &async_i2c1_irq_handler);
    i2c_init(i2c_inst, baudrate);
    i2c_set_slave_mode(i2c_inst, true, i2c_address);
    gpio_set_function(sda_pin, GPIO_FUNC_I2C);
    gpio_set_function(scl_pin, GPIO_FUNC_I2C);
    gpio_pull_up(sda_pin);
    gpio_pull_up(scl_pin);
    irq_set_exclusive_handler(irq_num, irq_handler);
    async_i2c_configure_interrupt_hw(i2c_inst);

    irq_set_enabled(irq_num, true);

    // Make the I2C pins available to picotool
    bi_decl_if_func_used(bi_2pins_with_func(sda_pin, scl_pin, GPIO_FUNC_I2C));

    async_i2c_target_initialized = true;
}