#pragma once

#include "CanmoreMsgClient.hpp"

class TitanMsgCanmoreClientHandler : public Canmore::ClientMsgHandler {
    virtual void handleMessage(uint8_t subtype, std::span<uint8_t> data) {}

    virtual void handleDecodeError(unsigned int errorCode) {}
};
