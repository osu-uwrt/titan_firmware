#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "hardware/irq.h"
#include "hardware/clocks.h"

#include "basic_queue/queue.h"

#include "can2040.h"
#include "can_bridge.h"
#include "canmore_msg_encoding.h"
#include "canmore_protocol.h"

// Variables shared between cores
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

static struct QUEUE_DEFINE(struct can2040_msg, 16) can_recv_queue = {0};

static canmore_msg_encoder_t msg_encoder;
static canmore_msg_decoder_t msg_decoder;


static void can2040_cb(struct can2040 *cd, uint32_t notify, struct can2040_msg *msg)
{
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
                // TODO: Note that a received message has been lost
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
        // TODO raise warning
    }
}

void process_can_frame(struct can2040_msg *msg) {
    canmore_id_t id = {.identifier = msg->id};

    // Decode incoming packet
    bool is_extended = ((msg->id & CAN2040_ID_EFF) != 0);

    if (msg->dlc > 8) {
        return;  // DLC greater than 8 isn't valid on canmore protocol
    }

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

    if (direction != CANMORE_DIRECTION_AGENT_TO_CLIENT) {
        // Received a frame that only this node is allowed to send?
        // TODO: Raise protocol warning
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
                }
            } else {
                canmore_msg_decode_reset_state(&msg_decoder); // Reset decoder state if not enough space for new message
                // TODO: Raise warning of message loss
            }
        }
    }
    else {  // type == CANMORE_TYPE_UTIL
        if (is_extended) {
            // Only standard frames allowed for utility messages
            // TODO: Raise protocol warning
            return;
        }

        if (!QUEUE_FULL(&utility_receive_queue)) {
            struct canmore_utility_frame *frame = QUEUE_CUR_WRITE_ENTRY(&utility_receive_queue);
            frame->channel = noc;
            frame->dlc = msg->dlc;
            frame->data32[0] = msg->data32[0];
            frame->data32[1] = msg->data32[1];
            QUEUE_MARK_WRITE_DONE(&utility_receive_queue);
        } else {
            // TODO: Raise warning of message loss
        }
    }
}

static struct can2040 cbus;

static void pio_irq_handler(void)
{
    can2040_pio_irq_handler(&cbus);
}

void core1_entry() {
    printf("Core 1 Startup\n");

    // Initialization
    {
        // Get the initial config, sent right after bringup
        const can_init_cfg_t *init_cfg = (const can_init_cfg_t*) multicore_fifo_pop_blocking();
        assigned_client_id = init_cfg->client_id;

        // TODO: Use gpio_term pin

        // Setup canbus
        can2040_setup(&cbus, init_cfg->pio_num);
        can2040_callback_config(&cbus, can2040_cb);

        // Setup irq
        uint irq_num = (init_cfg->pio_num == 0 ? PIO0_IRQ_0 : PIO1_IRQ_0);
        irq_set_exclusive_handler(irq_num, pio_irq_handler);
        irq_set_priority(irq_num, 0);

        // Start PIO machines
        can2040_start(&cbus, clock_get_hz(clk_sys), init_cfg->bitrate, init_cfg->gpio_rx, init_cfg->gpio_tx);

        // Start CAN bus for real (don't set this until fully set up or else nasty race conditions will occur)
        irq_set_enabled(irq_num, true);

        // TODO: Hook into safety before sending successful startup signal

        // init_cfg is no longer valid after sending startup response
        multicore_fifo_push_blocking(CORE1_STARTUP_DONE_FLAG);
    }

    // Configure heartbeat/CAN status detection
    heartbeat_msg.id = CANMORE_CALC_UTIL_ID_C2A(assigned_client_id, CANMORE_CHAN_HEARTBEAT);
    absolute_time_t next_heartbeat = make_timeout_time_ms(10);

    // Initialize CANmore Message Encoding objects
    canmore_msg_encode_init(&msg_encoder, assigned_client_id, CANMORE_DIRECTION_CLIENT_TO_AGENT);
    canmore_msg_decode_init(&msg_decoder);

    while (1) {
        // Process received CAN frames
        if (!QUEUE_EMPTY(&can_recv_queue)) {
            process_can_frame(QUEUE_CUR_READ_ENTRY(&can_recv_queue));
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

        // Send heartbeat
        if (time_reached(next_heartbeat) && can2040_check_transmit(&cbus)) {
            printf("Sent heartbeat\n");
            can2040_transmit(&cbus, &heartbeat_msg);
            // TODO: Fix heartbeat data
            heartbeat_msg.data[0]++;
            next_heartbeat = make_timeout_time_ms(1000);
        }

    }
}