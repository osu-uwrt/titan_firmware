#pragma once

#include "CanmoreMsgClient.hpp"
#include <utility>
#include <queue>

class TitanMsgLib {
private:
    std::queue<std::pair<uint8_t, std::span<uint8_t>>> msgs_to_send;

    TitanMsgLib();

public:
    static TitanMsgLib &getInstance();

    void init();

    void AddMsgToSend(uint8_t msg_type, std::span<uint8_t> data);
    void SendAll(Canmore::MsgClient& client);

    // Prevent copies
    TitanMsgLib(TitanMsgLib const&) = delete;
    void operator=(TitanMsgLib const&)  = delete;
};

class TitanMsgCanmoreClientHandler : public Canmore::ClientMsgHandler {
    virtual void handleMessage(uint8_t subtype, std::span<uint8_t> data);

    virtual void handleDecodeError(unsigned int errorCode);
};
