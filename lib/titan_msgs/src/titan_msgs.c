#include "titan/titan_msgs.h"

#include "pb_common.h"
#include "pb_decode.h"
#include "pb_encode.h"
#include "test.pb.h"

#include "pico/time.h"
#include "titan/list.h"

#define PROTOCOL_VERSION 1

#define MAX_PENDING_MSGS 16

#define PACKET_MAX_SIZE 128
#define NO_ACK 0

#define RX_BUF_SIZE 128

#define SEND_CONNECT_TIME_US (250 * 1000)

#define MAX_PROTOBUF_SIZE 256

/**
 * How this library works:
 *
 * State 1 (Connnecting):
 * - Every 250ms send a TITAN_CONNECT_REQ
 * - If TITAN_CONNECT_ACK with the latest ack number received, move to state 2
 *
 * State 2 (Operating):
 * - User can send and recieve unreliable/reliable messages
 * - User can receive a request via a callback must issue a response.
 */

// static struct LIST_DEFINE(struct pending_msg, MAX_PENDING_MSGS) pending_msgs = { 0 };

// static void clear_ack(uint32_t ack_num) {
//     for (size_t i = 0; i < LIST_LENGTH(&pending_msgs); i++) {
//         struct pending_msg msg = LIST_GET(&pending_msgs, i);
//         if (msg.ack_num == ack_num) {
//             on_ack_cb(msg.topic, true);
//             LIST_REMOVE(&pending_msgs, i);
//             return;
//         }
//     }

//     on_error_cb(TITAN_MSG_INVALID_ACK);
// }

static titan_msgs_on_recv_cb on_recv_cb = NULL;
static titan_msgs_on_error_cb on_error_cb = NULL;
static titan_msgs_on_req_cb on_req_cb = NULL;
static titan_msgs_transport_send transport_send = NULL;
static titan_msgs_transport_recv transport_recv = NULL;
static uint32_t subscribed_topics;
static uint32_t hosted_services;
static uint32_t board_id;

static bool connected = false;
static absolute_time_t last_connect_send;
static uint32_t last_connect_ack_num = 0;

static bool protobuf_send(message_type type, const void *msg, const pb_msgdesc_t *fields) {
    uint8_t data[MAX_PROTOBUF_SIZE];
    pb_ostream_t stream = pb_ostream_from_buffer(data, sizeof(data));
    if (!pb_encode(&stream, fields, &msg)) {
        return false;
    }

    transport_send(type, data, stream.bytes_written);

    return true;
}

static void recv_connect_ack(titan_ack ack) {
    if (connected) {
        // UH Oh :(
        return;
    }

    if (ack.ack == last_connect_ack_num) {
        connected = true;
    }
}

static void protobuf_recv(message_type type, const uint8_t *data, size_t data_len) {
    pb_istream_t stream = pb_istream_from_buffer(data, data_len);

    switch (type) {
    case message_type_TITAN_CONNECT_ACK: {
        titan_ack msg;
        pb_decode(&stream, titan_ack_fields, &msg);
        recv_connect_ack(msg);
        break;
    }
    default:
        return;  // TODO: handle error case
    }
}

void send_connect() {
    titan_connect_req req = (struct _titan_connect_req) {
        .ack = ++last_connect_ack_num,
        .subscribed_topics = subscribed_topics,
        .hosted_services = hosted_services,
        .version = PROTOCOL_VERSION,
        .board_id = board_id,
    };
    last_connect_send = get_absolute_time();

    protobuf_send(message_type_TITAN_CONNECT_REQ, &req, titan_connect_req_fields);
}

// =====================
//    PUBLIC METHODS
// =====================

void titan_msgs_init(struct titan_msgs_config cfg) {
    on_recv_cb = cfg.on_recv;
    on_req_cb = cfg.on_req;
    on_error_cb = cfg.on_error;
    transport_send = cfg.trans_send;
    transport_recv = cfg.trans_recv;
    board_id = cfg.board_id;

    send_connect();
}

void titan_msgs_tick() {
    // Parse messages that we have received
    uint8_t buf[MAX_PROTOBUF_SIZE];
    message_type recv_msg_type;
    size_t message_len = 0;
    while ((message_len = transport_recv(&recv_msg_type, buf, MAX_PROTOBUF_SIZE)) > 0) {
        protobuf_recv(recv_msg_type, buf, message_len);
    }

    // Send a connect message if neccessary
    if (!connected) {
        if (absolute_time_diff_us(get_absolute_time(), last_connect_send) > SEND_CONNECT_TIME_US) {
            send_connect();
        }
    }
}

void titan_msgs_send(titan_msg *msg) {
    if (connected) {
        protobuf_send(message_type_TITAN_UNRELIABLE, msg, titan_unreliable_fields);
    }
}

void titan_msgs_send_with_ack(titan_msg *msg, uint64_t timeout) {
    if (!connected) {
        return;
    }
    (void) msg;
    (void) timeout;
    // if(connected) { send(); }
}
