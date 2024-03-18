#ifndef TITAN_MSGS_H
#define TITAN_MSGS_H

#include "test.pb.h"

enum titan_msgs_error {
    // Occurs when the board receives an acknowledge when it did not request one
    TITAN_MSG_INVALID_ACK = 1,
    // TODO define more
};

typedef void (*titan_msgs_on_error_cb)(enum titan_msgs_error error);

typedef void (*titan_msgs_on_recv_cb)(titan_msg msg);

// return if the response should be sent
typedef bool (*titan_msgs_on_req_cb)(titan_request_payload request, titan_response_payload *response);

typedef bool (*titan_msgs_transport_send)(uint8_t msg_type, uint8_t *buf, size_t len);

typedef size_t (*titan_msgs_transport_recv)(uint8_t *msg_type, uint8_t *buf, size_t max_len);

struct titan_msgs_config {
    titan_msgs_on_recv_cb on_recv;
    titan_msgs_on_req_cb on_req;
    titan_msgs_on_error_cb on_error;
    titan_msgs_transport_send trans_send;
    titan_msgs_transport_recv trans_recv;
    uint32_t board_id;  // TODO - do we need this...? if we do what size?
    uint32_t subscribed_topics;
    uint32_t hosted_services;
};

void titan_msgs_init(struct titan_msgs_config cfg);

void titan_msgs_tick();

void titan_msg_send_unreliable(titan_msg msg);

void titan_msg_send_reliable(titan_msg msg);

#endif
