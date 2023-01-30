#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "hardware/clocks.h"
#include "hardware/irq.h"
#include "hardware/gpio.h"

#include "basic_queue/queue.h"

#include "mcp2515/mcp2515.h"
#include "can_bridge.h"
#include "canmore/msg_encoding.h"
#include "canmore/protocol.h"

// ========================================
// Global Definitions
// ========================================

// Variables shared between cores
volatile bool canbus_device_in_error_state = false;
absolute_time_t canbus_heartbeat_timeout = {0};  // evaluates to nil_time
struct utility_receive_queue utility_receive_queue = {0};
struct utility_transmit_queue utility_transmit_queue = {0};
struct msg_receive_queue msg_receive_queue = {0};
struct msg_transmit_queue msg_transmit_queue = {0};

// Prebuilt heartbeat message
static struct can_frame heartbeat_msg = {
    // .id to be defined during initialization
    .can_dlc = 1,
    .data = {0}
};

// The client id for this can node/device
static uint8_t assigned_client_id;

static struct QUEUE_DEFINE(struct can_frame, CAN_FRAME_RX_QUEUE_SIZE) can_recv_queue = {0};

static canmore_msg_encoder_t msg_encoder;
static canmore_msg_decoder_t msg_decoder;


// ========================================
// Error Reporting
// ========================================
/**
 * @brief Write to SIO FIFO without checking if full.
 * Important for error routines where it could be in an interrupt and shouldn't block if the first core is slow
 *
 * @param data Data to write to fifo
 */
static __force_inline void write_fifo_noblock(uint32_t data) {
    sio_hw->fifo_wr = data;
    __sev();
}

// static void can_report_receive_error(void){
//     write_fifo_noblock(SIO_ERROR_RECEIVE_FLAG);
// }
#define can_report_receive_error() do {write_fifo_noblock(SIO_ERROR_RECEIVE_FLAG + __LINE__);} while(0)
#define can_report_internal_error() do {write_fifo_noblock(SIO_ERROR_INTERNAL_FLAG + __LINE__);} while(0)

// static void can_report_internal_error(void){
//     write_fifo_noblock(SIO_ERROR_INTERNAL_FLAG);
// }


// ========================================
// CAN2040 IRQ Handler
// ========================================

MCP2515 *mcp2515_ptr = NULL;

static void can_int_irq_handler(__unused uint gpio, __unused uint32_t events) {
    // Handle incoming packets
    struct can_frame rx;

    while (mcp2515_ptr->readMessage(&rx) == MCP2515::ERROR_OK) {
        canmore_id_t id = {.identifier = rx.can_id};
        bool is_extended = ((rx.can_id & CAN_EFF_FLAG) != 0);
        uint8_t client_id = (is_extended ? id.pkt_ext.client_id : id.pkt_std.client_id);

        // Copy message for processing in non-criticial section
        if (client_id == assigned_client_id) {
            if (!QUEUE_FULL(&can_recv_queue)) {
                memcpy(QUEUE_CUR_WRITE_ENTRY(&can_recv_queue), &rx, sizeof(struct can_frame));
                QUEUE_MARK_WRITE_DONE(&can_recv_queue);
            } else {
                // Internal process queue overflow
                can_report_internal_error();
            }
        }
    }

    if (mcp2515_ptr->getErrorFlags() & (MCP2515::EFLG_RX0OVR | MCP2515::EFLG_RX1OVR)) {
        // This notify is only called when the PIO rx reports overflow
        can_report_internal_error();
        mcp2515_ptr->clearRXnOVR();
    }

    mcp2515_ptr->clearMERR();
    mcp2515_ptr->clearERRIF();
}


// ========================================
// Frame Decoding
// ========================================

void can_decode_error_callback(__unused void* arg, __unused unsigned int error_code) {
    // If the decoder doesn't like it, it's a receive error
    //can_report_receive_error();
    write_fifo_noblock(SIO_ERROR_RECEIVE_FLAG + 70000 + error_code);
}

void can_process_frame(struct can_frame *msg) {
    canmore_id_t id = {.identifier = msg->can_id};

    // Decode incoming packet
    bool is_extended = ((msg->can_id & CAN_EFF_FLAG) != 0);

    uint32_t client_id;
    uint32_t type;
    uint32_t direction;
    uint32_t noc;
    uint32_t crc;

    if (is_extended) {
        client_id = id.pkt_ext.client_id;
        type = id.pkt_ext.type;
        direction = id.pkt_ext.direction;
        noc = id.pkt_ext.noc;
        crc = id.pkt_ext.crc;
    } else {
        client_id = id.pkt_std.client_id;
        type = id.pkt_std.type;
        direction = id.pkt_std.direction;
        noc = id.pkt_std.noc;
        crc = 0;
    }

    if (client_id != assigned_client_id) {
        // Message not for this client, ignore it
        return;
    }

    if (msg->can_dlc > CANMORE_FRAME_SIZE) {
        // DLC greater than 8 isn't valid on canmore protocol
        can_report_receive_error();
        return;
    }

    if (msg->can_id & CAN_RTR_FLAG) {
        // CANmore does not support RTR frames
        can_report_receive_error();
        return;
    }

    if (direction != CANMORE_DIRECTION_AGENT_TO_CLIENT) {
        // Received a frame that only this node is allowed to send?
        can_report_receive_error();
        return;
    }

    if (type == CANMORE_TYPE_MSG) {
        if (!is_extended) {
            canmore_msg_decode_frame(&msg_decoder, noc, msg->data, msg->can_dlc);
        } else {
            // Only copy into queue if there's space
            if (!QUEUE_FULL(&msg_receive_queue)) {
                struct canmore_msg *entry = QUEUE_CUR_WRITE_ENTRY(&msg_receive_queue);
                size_t msg_size = canmore_msg_decode_last_frame(&msg_decoder, noc, msg->data, msg->can_dlc, crc, entry->data);

                // Only queue the data if the decode was successful
                if (msg_size > 0) {
                    entry->length = msg_size;
                    QUEUE_MARK_WRITE_DONE(&msg_receive_queue);
                    write_fifo_noblock(SIO_RECV_MESSAGE_FLAG);
                }
            } else {
                // Message queue overflow (primary core isn't reading fast enough)
                canmore_msg_decode_reset_state(&msg_decoder);
                can_report_internal_error();
            }
        }
    }
    else {  // type == CANMORE_TYPE_UTIL
        if (is_extended) {
            // Only standard frames allowed for utility messages
            can_report_receive_error();
            return;
        }

        if (noc == CANMORE_CHAN_HEARTBEAT) {
            // CANmore does not permit agent-to-client communication over the heartbeat channel
            can_report_receive_error();
            return;
        }

        if (!QUEUE_FULL(&utility_receive_queue)) {
            struct canmore_utility_frame *frame = QUEUE_CUR_WRITE_ENTRY(&utility_receive_queue);
            frame->channel = noc;
            frame->dlc = msg->can_dlc;
            memcpy(frame->data, msg->data, sizeof(msg->data));
            QUEUE_MARK_WRITE_DONE(&utility_receive_queue);
            write_fifo_noblock(SIO_RECV_UTILITY_FRAME_FLAG);
        } else {
            // Utility frame queue overflow (primary core isn't reading fast enough)
            can_report_internal_error();
        }
    }
}


// ========================================
// Main Run Loop
// ========================================

void core1_entry() {
    // Initialization

    // Get the initial config, sent right after bringup
    const can_init_cfg_t *init_cfg = (const can_init_cfg_t*) multicore_fifo_pop_blocking();
    assigned_client_id = init_cfg->client_id;

    // Calculate heartbeat ID with new client id
    heartbeat_msg.can_id = CANMORE_CALC_UTIL_ID_C2A(assigned_client_id, CANMORE_CHAN_HEARTBEAT);

    int gpio_term_pin = init_cfg->gpio_term;
    if (gpio_term_pin >= 0) {
        gpio_init(gpio_term_pin);
        gpio_pull_up(gpio_term_pin);
        gpio_set_dir(gpio_term_pin, false);
    }

    // Initialize CANmore Message Encoder/Decoder objects
    canmore_msg_encode_init(&msg_encoder, assigned_client_id, CANMORE_DIRECTION_CLIENT_TO_AGENT);
    canmore_msg_decode_init(&msg_decoder, can_decode_error_callback, NULL);

    // Setup canbus
    MCP2515 mcp2515(init_cfg->spi_channel, init_cfg->cs_pin, init_cfg->mosi_pin, init_cfg->miso_pin, init_cfg->sck_pin, init_cfg->spi_clock);
    mcp2515_ptr = &mcp2515;
    mcp2515.reset();
    mcp2515.setBitrate(CAN_1000KBPS, MCP_16MHZ);
    mcp2515.setNormalMode();

    // Start CAN bus for real (don't set this until fully set up or else nasty race conditions will occur)
    gpio_init(init_cfg->int_pin);
    gpio_set_dir(init_cfg->int_pin, GPIO_IN);
    gpio_pull_up(init_cfg->int_pin);
    gpio_set_irq_enabled_with_callback(init_cfg->int_pin, GPIO_IRQ_LEVEL_LOW, true, &can_int_irq_handler);

    // Mark Startup Complete
    // init_cfg is no longer valid after sending startup response
    multicore_fifo_push_blocking(SIO_STARTUP_DONE_FLAG);


    absolute_time_t next_heartbeat = make_timeout_time_ms(10);
    absolute_time_t next_alive_flag = get_absolute_time();

    while (1) {
        // Process received CAN frames
        if (!QUEUE_EMPTY(&can_recv_queue)) {
            can_process_frame(QUEUE_CUR_READ_ENTRY(&can_recv_queue));
            QUEUE_MARK_READ_DONE(&can_recv_queue);
        }

        // Process CANmore utility frames to transmit
        if (!QUEUE_EMPTY(&utility_transmit_queue) && mcp2515.checkTransmit(MCP2515::TXB0)) {
            struct can_frame tx_msg;

            struct canmore_utility_frame *frame = QUEUE_CUR_READ_ENTRY(&utility_transmit_queue);
            memcpy(tx_msg.data, frame->data, sizeof(tx_msg.data));
            tx_msg.can_dlc = frame->dlc;
            tx_msg.can_id = CANMORE_CALC_UTIL_ID_C2A(assigned_client_id, frame->channel);
            QUEUE_MARK_READ_DONE(&utility_transmit_queue);

            mcp2515.sendMessage(MCP2515::TXB0, &tx_msg);
        }

        // Process CANmore messages to transmit
        if (!QUEUE_EMPTY(&msg_transmit_queue) && canmore_msg_encode_done(&msg_encoder)) {
            struct canmore_msg *entry = QUEUE_CUR_READ_ENTRY(&msg_transmit_queue);
            canmore_msg_encode_load(&msg_encoder, entry->data, entry->length);
            QUEUE_MARK_READ_DONE(&msg_transmit_queue);
        }

        while (!canmore_msg_encode_done(&msg_encoder) && mcp2515.checkTransmit(MCP2515::TXB0)) {
            struct can_frame tx_msg;
            bool is_extended;
            canmore_msg_encode_next(&msg_encoder, tx_msg.data, &tx_msg.can_dlc, &tx_msg.can_id, &is_extended);

            if (is_extended) {
                tx_msg.can_id |= CAN_EFF_FLAG;
            }

            mcp2515.sendMessage(MCP2515::TXB0, &tx_msg);
        }

        // Send heartbeat frame
        if (time_reached(next_heartbeat) && mcp2515.checkTransmit(MCP2515::TXB0)) {
            canmore_heartbeat_t last_heartbeat = {.data = heartbeat_msg.data[0]};

            // Extra data in heartbeat contains termination pin info
            // Bit 0 of extra is if termination data exists
            // Bit 1 of extra is state of termination (0 not terminated, 1 is terminated)
            uint term_data = 0;
            if (gpio_term_pin >= 0) {
                term_data = (gpio_get(gpio_term_pin) ? 3 : 1);
            }

            heartbeat_msg.data[0] = CANMORE_CALC_HEARTBEAT_DATA(last_heartbeat.pkt.cnt + 1, canbus_device_in_error_state, term_data);

            if (mcp2515.sendMessage(MCP2515::TXB0, &heartbeat_msg) == MCP2515::ERROR_OK) {
                canbus_heartbeat_timeout = make_timeout_time_ms(CAN_HEARTBEAT_TIMEOUT_MS);
            }

            next_heartbeat = make_timeout_time_ms(CAN_HEARTBEAT_INTERVAL_MS);
        }

        // Send core alive flag
        if (time_reached(next_alive_flag)) {
            write_fifo_noblock(SIO_CORE_ALIVE_FLAG);
            next_alive_flag = make_timeout_time_ms(CAN_CORE_ALIVE_INTERVAL_MS);
        }
    }
}