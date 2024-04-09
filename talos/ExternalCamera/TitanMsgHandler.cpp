#include "TitanMsgHandler.hpp"
#include <chrono>

extern "C" {
#include "titan/titan_msgs.h"
}

static uint64_t get_time_ms_cb() {
    auto time_since_epoch = std::chrono::system_clock::now().time_since_epoch();
    auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(time_since_epoch);

    return ms.count();
}

static bool transport_send_cb(uint8_t msg_type, uint8_t *buf, size_t len) {
    uint8_t *data = (uint8_t*) malloc(len);
    memcpy(data, buf, len);
    std::span span(data, len);

    TitanMsgLib::getInstance().AddMsgToSend(msg_type, span);

    return true;
}

TitanMsgLib &TitanMsgLib::getInstance() {
    static TitanMsgLib lib;

    return lib;
}

void TitanMsgLib::init() {
    struct titan_msgs_config cfg = (struct titan_msgs_config) {
        .on_recv = NULL,
        .on_req = NULL,
        .on_error = NULL,
        .trans_send = transport_send_cb,
        .trans_recv = NULL,
        .get_time_ms_cb = get_time_ms_cb,
        .board_id = 0,
        .subscribed_topics = 0,
        .hosted_services = 0,
    };

    titan_msgs_init(cfg);
}

TitanMsgLib::TitanMsgLib() {}

void TitanMsgLib::AddMsgToSend(uint8_t msg_type, std::span<uint8_t> data) {
    std::pair<uint8_t, std::span<uint8_t>> pair(msg_type, data);

    printf("Send %d\n", msg_type);
    this->msgs_to_send.push(pair);
}

void TitanMsgLib::SendAll(Canmore::MsgClient& client) {
    while(!this->msgs_to_send.empty()) {
        auto msg_to_send = this->msgs_to_send.front();
        client.transmitMessage(msg_to_send.first, msg_to_send.second);
        this->msgs_to_send.pop();
    }
}

void TitanMsgCanmoreClientHandler::handleDecodeError(unsigned int errorCode) {}

void TitanMsgCanmoreClientHandler::handleMessage(uint8_t subtype, std::span<uint8_t> data) {}

// Make handler a singleton?
