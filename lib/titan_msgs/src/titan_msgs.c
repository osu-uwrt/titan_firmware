#include "titan/titan_msgs.h"

#include "pb_common.h"
#include "pb_decode.h"
#include "pb_encode.h"
#include "pico.h"

#include "pico/time.h"
#include "titan/list.h"

#define MAX_PENDING_MSGS 16

#define PACKET_MAX_SIZE 128
#define NO_ACK 0

#define RX_BUF_SIZE 128

static titan_msgs_on_recv_cb on_recv_cb = NULL;
static titan_msgs_on_error_cb on_error_cb = NULL;
static titan_msgs_on_ack on_ack_cb = NULL;
static titan_msgs_transport_send transport_send = NULL;
static titan_msgs_transport_recv transport_recv = NULL;

static int next_ack_id = 1;

struct pending_msg {
    titan_fw_comm_msg_topic_types topic;
    uint32_t ack_num;
    absolute_time_t timeout;
};

static struct LIST_DEFINE(struct pending_msg, MAX_PENDING_MSGS) pending_msgs = { 0 };

static void clear_ack(uint32_t ack_num) {
    for (size_t i = 0; i < LIST_LENGTH(&pending_msgs); i++) {
        struct pending_msg msg = LIST_GET(&pending_msgs, i);
        if (msg.ack_num == ack_num) {
            on_ack_cb(msg.topic, true);
            LIST_REMOVE(&pending_msgs, i);
            return;
        }
    }

    on_error_cb(TITAN_MSG_INVALID_ACK);
}

static void check_acks() {
    for (size_t i = 0; i < LIST_LENGTH(&pending_msgs); i++) {
        struct pending_msg msg = LIST_GET(&pending_msgs, i);
        if (time_reached(LIST_GET(&pending_msgs, i).timeout)) {
            on_ack_cb(msg.topic, false);
            LIST_REMOVE(&pending_msgs, i);
            // Note: We need to check acks again. Since we modified the array
            // we can just start checking again at the start. This is not very
            // efficient, but this array should not be very long.
            check_acks();
        }
    }
}

void titan_msgs_init(titan_msgs_on_recv_cb on_recv, titan_msgs_on_error_cb on_error, titan_msgs_on_ack on_ack,
                     titan_msgs_transport_send trans_send, titan_msgs_transport_recv trans_recv) {
    on_recv_cb = on_recv;
    on_error_cb = on_error;
    on_ack_cb = on_ack;
    transport_send = trans_send;
    transport_recv = trans_recv;
}

void titan_msgs_tick() {
    uint8_t rx_buf[RX_BUF_SIZE];
    size_t length_read = 0;
    titan_fw_comm_msg msg;
    while ((length_read = transport_recv(rx_buf, RX_BUF_SIZE)) != 0) {
        pb_istream_t stream = pb_istream_from_buffer(rx_buf, length_read);
        pb_decode(&stream, titan_fw_comm_msg_fields, &msg);

        if (msg.ack != NO_ACK) {
            clear_ack(msg.ack);
        }

        on_recv_cb(&msg);
    }

    check_acks();
}

void titan_msgs_send(titan_fw_comm_msg *msg) {
    uint8_t buf[PACKET_MAX_SIZE];
    pb_ostream_t stream = pb_ostream_from_buffer(buf, PACKET_MAX_SIZE);
    if (pb_encode(&stream, titan_fw_comm_msg_fields, msg)) {
        transport_send(buf, stream.bytes_written);
    }
}

void titan_msgs_send_with_ack(titan_fw_comm_msg *msg, uint64_t timeout) {
    msg->ack = next_ack_id++;
    titan_msgs_send(msg);

    struct pending_msg pending_msg = {0};
    pending_msg.ack_num = msg->ack;
    pending_msg.timeout = make_timeout_time_ms(timeout);
    pending_msg.topic = msg->topic;

    LIST_ADD(&pending_msgs, pending_msg);
}
