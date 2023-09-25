#ifndef TITAN_MSGS_H
#define TITAN_MSGS_H

#include "test.pb.h"

enum titan_msgs_error {
    // Occurs when the board receives an acknowledge when it did not request one
    TITAN_MSG_INVALID_ACK = 1,
    // TODO define more
};

typedef void (*titan_msgs_on_error_cb)(enum titan_msgs_error error);

typedef void (*titan_msgs_on_recv_cb)(titan_fw_comm_msg *msg);

typedef void (*titan_msgs_on_ack)(titan_fw_comm_msg_topic_types topic, bool ack_recv);

typedef bool (*titan_msgs_transport_send)(uint8_t *buf, size_t len);

typedef size_t (*titan_msgs_transport_recv)(uint8_t *buf, size_t max_len);

void titan_msgs_init(titan_msgs_on_recv_cb on_recv, titan_msgs_on_error_cb on_error, titan_msgs_on_ack on_ack,
                     titan_msgs_transport_send trans_send, titan_msgs_transport_recv trans_recv);

void titan_msgs_tick();

void titan_msgs_send(titan_fw_comm_msg *msg);

void titan_msgs_send_with_ack(titan_fw_comm_msg *msg, uint64_t timeout_ms);

#endif
