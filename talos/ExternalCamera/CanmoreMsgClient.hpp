#pragma once

#include "canmore_cpp/CANSocket.hpp"

#include "canmore/msg_encoding.h"

#include <span>

namespace Canmore {

class MsgHandler {
public:
    virtual void handleMessage(uint8_t subtype, std::span<uint8_t> data) = 0;
    virtual void handleDecodeError(unsigned int errorCode) = 0;
};

/**
 * @brief Class to act as a CANmore Client for Message frames.
 *
 * This initializes a CAN socket to handle receiving and transmitting the message frames, and provides
 * a simple function to transmit new messages, and users can inherit the MsgHandler class to receive messages
 * or report decode errors.
 */
class MsgClient : public CANSocket {
public:
    /**
     * @brief Creates a new CANmore Message Client
     *
     * @param ifIndex The CAN interface index for the client to bind to
     * @param clientId The client ID for this client on the CAN bus
     * @param handler Class to handle incoming messages or report errors
     */
    MsgClient(int ifIndex, uint8_t clientId, MsgHandler &handler);

    /**
     * @brief Transmits a new message frame
     *
     * @param subtype The subtype for the message frame (see CANmore Specification)
     * @param data The data for the message to transmit
     */
    void transmitMessage(uint8_t subtype, std::span<uint8_t> data);

    const uint8_t clientId;

protected:
    /*
     * Overrides for CANSocket
     */
    void handleFrame(canid_t can_id, const std::span<const uint8_t> &data) override;

private:
    static void decoderErrorCallback(void *arg, unsigned int errorCode) {
        auto client = (MsgClient *) arg;
        client->handler.handleDecodeError(errorCode);
    }

    std::vector<uint8_t> handlerBuffer;
    MsgHandler &handler;
    canmore_msg_encoder_t encoder;
    canmore_msg_decoder_t decoder;
};

};  // namespace Canmore
