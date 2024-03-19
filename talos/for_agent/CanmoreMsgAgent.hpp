#pragma once

#include "canmore_cpp/CANSocket.hpp"

#include "canmore/msg_encoding.h"

#include <list>
#include <vector>

namespace Canmore {

/**
 * @brief Abstract handler class to be passed to the MsgAgent for responding to events
 */
class AgentMsgHandler {
public:
    /**
     * @brief Callback for when a new message is received by the MsgAgent class
     *
     * @param clientId The client that sent the message
     * @param subtype The subtype for the message
     * @param data Contents of the message
     */
    virtual void handleMessage(uint8_t clientId, uint8_t subtype, std::span<uint8_t> data) = 0;

    /**
     * @brief Callback for when a message frame fails to decode
     *
     * @param clientId The client that sent the erronious message
     * @param errorCode The error code for the specific error. Use MsgAgent::lookupDecodeError to get a description
     */
    virtual void handleDecodeError(uint8_t clientId, unsigned int errorCode) = 0;

    /**
     * @brief Callback for when traffic is detected from another agent on the network.
     */
    virtual void handleConflictingAgentError() {}
};

/**
 * @brief CANmore Message Agent class to handle communication with all CANmore clients on the network using messages.
 */
class MsgAgent : public CANSocket {
public:
    /**
     * @brief Construct a new Message Agent object
     *
     * @param ifIndex The network interface index to bind to. Must be a CAN Bus interface
     * @param handler The handler class all callbacks will be sent to
     */
    MsgAgent(int ifIndex, AgentMsgHandler &handler);

    /**
     * @brief Transmits a new canmore message
     *
     * @param clientId The destination client ID, or 0 for broadcast
     * @param subtype The subtype for the message (see CANmore Specification)
     * @param data The message data to transmit
     */
    void transmitMessage(uint8_t clientId, uint8_t subtype, std::span<uint8_t> data);

    /**
     * @brief Looks up a decode error code to a human readable string
     *
     * @param errorCode The error code to look up
     * @param strOut Reference to write the resulting error code
     */
    static void lookupDecodeError(unsigned int errorCode, std::string &strOut);

protected:
    /*
     * Overrides for CANSocket
     */
    void handleFrame(canid_t can_id, const std::span<const uint8_t> &data) override;

private:
    typedef std::pair<MsgAgent *, uint8_t> DecodeErrorCbArg;  // Format: {Agent Instance, Client ID}

    // Static function to call appropriate decode error handler for the specific decoder
    static void decoderErrorCallback(void *arg, unsigned int errorCode) {
        auto argData = (DecodeErrorCbArg *) arg;
        argData->first->handler.handleDecodeError(argData->second, errorCode);
    }

    AgentMsgHandler &handler;                      // Handler for this class
    std::vector<canmore_msg_encoder_t> encoders;   // Array of encoders for connected clients (index is client id)
    std::list<DecodeErrorCbArg> decoderErrorArgs;  // Holds args for decode error callbacks (refs must stay constant)
    std::vector<canmore_msg_decoder_t> decoders;   // Array of decoders for connected clients (index is client id - 1)
};

}  // namespace Canmore
