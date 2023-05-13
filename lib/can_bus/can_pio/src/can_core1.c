#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include "pico/multicore.h"
#include "hardware/irq.h"
#include "hardware/clocks.h"

#include "basic_queue/queue.h"

#include "can2040.h"
#include "can_bridge.h"
#include "canmore/msg_encoding.h"
#include "canmore/protocol.h"

// ========================================
// Global Definitions
// ========================================

// Variables shared between cores
volatile bool canbus_device_in_error_state = false;
volatile absolute_time_t canbus_heartbeat_timeout = {0};  // evaluates to nil_time
struct utility_receive_queue utility_receive_queue = {0};
struct utility_transmit_queue utility_transmit_queue = {0};
struct msg_receive_queue msg_receive_queue = {0};
struct msg_transmit_queue msg_transmit_queue = {0};

// Prebuilt heartbeat message
static struct can2040_msg heartbeat_msg = {
    // .id to be defined during initialization
    .dlc = 1,
    .data = {0}
};

// The client id for this can node/device
static uint8_t assigned_client_id;

static struct QUEUE_DEFINE(struct can2040_msg, CAN_FRAME_RX_QUEUE_SIZE) can_recv_queue = {0};

static struct can2040 cbus;
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

static void __can2040_irq_func(can2040_cb)(__unused struct can2040 *cd, uint32_t notify, struct can2040_msg *msg) {
    // Handle incoming packets
    if (notify == CAN2040_NOTIFY_RX) {
        canmore_id_t id = {.identifier = msg->id};
        bool is_extended = ((msg->id & CAN2040_ID_EFF) != 0);
        uint8_t client_id = (is_extended ? id.pkt_ext.client_id : id.pkt_std.client_id);

        // Copy message for processing in non-criticial section
        if (client_id == assigned_client_id) {
            if (!QUEUE_FULL(&can_recv_queue)) {
                memcpy(QUEUE_CUR_WRITE_ENTRY(&can_recv_queue), msg, sizeof(struct can2040_msg));
                QUEUE_MARK_WRITE_DONE(&can_recv_queue);
            } else {
                // Internal process queue overflow
                can_report_internal_error();
            }
        }
    }

    // Handle any callbacks for transmitted frames
    if (notify == CAN2040_NOTIFY_TX) {
        if (msg->id == heartbeat_msg.id) {
            canbus_heartbeat_timeout = make_timeout_time_ms(CAN_HEARTBEAT_TIMEOUT_MS);
        }
    }

    if (notify == CAN2040_NOTIFY_ERROR) {
        // This notify is only called when the PIO rx reports overflow
        can_report_internal_error();
    }
}

static void __can2040_irq_func(can_pio_irq_handler)(void) {
    can2040_pio_irq_handler(&cbus);
}


// ========================================
// Frame Decoding
// ========================================

void can_decode_error_callback(__unused void* arg, __unused unsigned int error_code) {
    // If the decoder doesn't like it, it's a receive error
    //can_report_receive_error();
    write_fifo_noblock(SIO_ERROR_RECEIVE_FLAG + 70000 + error_code);
}

void can_process_frame(struct can2040_msg *msg) {
    canmore_id_t id = {.identifier = msg->id};

    // Decode incoming packet
    bool is_extended = ((msg->id & CAN2040_ID_EFF) != 0);

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

    if (msg->dlc > CANMORE_FRAME_SIZE) {
        // DLC greater than 8 isn't valid on canmore protocol
        can_report_receive_error();
        return;
    }

    if (msg->id & CAN2040_ID_RTR) {
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
            canmore_msg_decode_frame(&msg_decoder, noc, msg->data, msg->dlc);
        } else {
            // Only copy into queue if there's space
            if (!QUEUE_FULL(&msg_receive_queue)) {
                struct canmore_msg *entry = QUEUE_CUR_WRITE_ENTRY(&msg_receive_queue);
                size_t msg_size = canmore_msg_decode_last_frame(&msg_decoder, noc, msg->data, msg->dlc, crc, entry->data);

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
            frame->dlc = msg->dlc;
            frame->data32[0] = msg->data32[0];
            frame->data32[1] = msg->data32[1];
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
    int gpio_term_pin = -1;

    // Initialization
    {
        // Get the initial config, sent right after bringup
        const can_init_cfg_t *init_cfg = (const can_init_cfg_t*) multicore_fifo_pop_blocking();
        assigned_client_id = init_cfg->client_id;

        // Calculate heartbeat ID with new client id
        heartbeat_msg.id = CANMORE_CALC_UTIL_ID_C2A(assigned_client_id, CANMORE_CHAN_HEARTBEAT);

        gpio_term_pin = init_cfg->gpio_term;
        if (gpio_term_pin >= 0) {
            gpio_init(gpio_term_pin);
            gpio_pull_up(gpio_term_pin);
            gpio_set_dir(gpio_term_pin, false);
        }

        // Initialize CANmore Message Encoder/Decoder objects
        canmore_msg_encode_init(&msg_encoder, assigned_client_id, CANMORE_DIRECTION_CLIENT_TO_AGENT);
        canmore_msg_decode_init(&msg_decoder, can_decode_error_callback, NULL);

        // Setup canbus
        can2040_setup(&cbus, init_cfg->pio_num);
        can2040_callback_config(&cbus, can2040_cb);

        uint irq_num = (init_cfg->pio_num == 0 ? PIO0_IRQ_0 : PIO1_IRQ_0);
        irq_set_exclusive_handler(irq_num, can_pio_irq_handler);
        irq_set_priority(irq_num, 0);

        // Start PIO machines
        can2040_start(&cbus, clock_get_hz(clk_sys), init_cfg->bitrate, init_cfg->gpio_rx, init_cfg->gpio_tx);

        // Mark Startup Complete
        // init_cfg is no longer valid after sending startup response
        multicore_fifo_push_blocking(SIO_STARTUP_DONE_FLAG);

        // Start CAN bus for real (don't set this until fully set up or else nasty race conditions will occur)
        irq_set_enabled(irq_num, true);
    }

    absolute_time_t next_heartbeat = make_timeout_time_ms(10);
    absolute_time_t next_alive_flag = get_absolute_time();

    while (1) {
        // Process received CAN frames
        if (!QUEUE_EMPTY(&can_recv_queue)) {
            can_process_frame(QUEUE_CUR_READ_ENTRY(&can_recv_queue));
            QUEUE_MARK_READ_DONE(&can_recv_queue);
        }

        // Process CANmore utility frames to transmit
        if (!QUEUE_EMPTY(&utility_transmit_queue) && can2040_check_transmit(&cbus)) {
            struct can2040_msg tx_msg;

            struct canmore_utility_frame *frame = QUEUE_CUR_READ_ENTRY(&utility_transmit_queue);
            tx_msg.data32[0] = frame->data32[0];
            tx_msg.data32[1] = frame->data32[1];
            tx_msg.dlc = frame->dlc;
            tx_msg.id = CANMORE_CALC_UTIL_ID_C2A(assigned_client_id, frame->channel);
            QUEUE_MARK_READ_DONE(&utility_transmit_queue);

            can2040_transmit(&cbus, &tx_msg);
        }

        // Process CANmore messages to transmit
        if (!QUEUE_EMPTY(&msg_transmit_queue) && canmore_msg_encode_done(&msg_encoder)) {
            struct canmore_msg *entry = QUEUE_CUR_READ_ENTRY(&msg_transmit_queue);
            canmore_msg_encode_load(&msg_encoder, entry->data, entry->length);
            QUEUE_MARK_READ_DONE(&msg_transmit_queue);
        }

        while (!canmore_msg_encode_done(&msg_encoder) && can2040_check_transmit(&cbus)) {
            struct can2040_msg tx_msg;
            bool is_extended;
            canmore_msg_encode_next(&msg_encoder, tx_msg.data, &tx_msg.dlc, &tx_msg.id, &is_extended);

            if (is_extended) {
                tx_msg.id |= CAN2040_ID_EFF;
            }

            can2040_transmit(&cbus, &tx_msg);
        }

        // Send heartbeat frame
        if (time_reached(next_heartbeat) && can2040_check_transmit(&cbus)) {
            canmore_heartbeat_t last_heartbeat = {.data = heartbeat_msg.data[0]};

            // Extra data in heartbeat contains termination pin info
            // Bit 0 of extra is if termination data exists
            // Bit 1 of extra is state of termination (0 not terminated, 1 is terminated)
            uint term_data = 0;
            if (gpio_term_pin >= 0) {
                term_data = (gpio_get(gpio_term_pin) ? 3 : 1);
            }

            heartbeat_msg.data[0] = CANMORE_CALC_HEARTBEAT_DATA(last_heartbeat.pkt.cnt + 1, canbus_device_in_error_state, term_data);
            can2040_transmit(&cbus, &heartbeat_msg);

            next_heartbeat = make_timeout_time_ms(CAN_HEARTBEAT_INTERVAL_MS);
        }

        // Send core alive flag
        if (time_reached(next_alive_flag)) {
            write_fifo_noblock(SIO_CORE_ALIVE_FLAG);
            next_alive_flag = make_timeout_time_ms(CAN_CORE_ALIVE_INTERVAL_MS);
        }
    }
}