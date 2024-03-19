#include "CanmoreMsgAgent.hpp"

using namespace Canmore;

MsgAgent::MsgAgent(int ifIndex, AgentMsgHandler &handler): CANSocket(ifIndex), handler(handler) {
    // Setup Receive Filter
    // Need to match both standard CAN frames and extended CAN message frames from all clients and agents
    // We're subscribed to agent messages as well since we should be the only agent on the network
    // If we see any packets from another agent, we can report an error
    struct can_filter rfilter[] = {
        { .can_id = CANMORE_CALC_MSG_ID(0, 0, 0),
          .can_mask = (CAN_EFF_FLAG | CAN_RTR_FLAG | CANMORE_CALC_FILTER_MASK(0, 1, 0, 0)) },
        { .can_id = CAN_EFF_FLAG | CANMORE_CALC_MSG_EXT_ID(0, 0, 0, 0),
          .can_mask = (CAN_EFF_FLAG | CAN_RTR_FLAG | CANMORE_CALC_EXT_FILTER_MASK(0, 1, 0, 0, 0)) },
    };
    setRxFilters(std::span { rfilter });
}

void MsgAgent::transmitMessage(uint8_t clientId, uint8_t subtype, std::span<uint8_t> data) {
    if (subtype >= (1 << CANMORE_MSG_SUBTYPE_LENGTH)) {
        throw std::logic_error("Attempting to transmit canmore message with invalid message subtype");
    }
    if (data.size() > CANMORE_MAX_MSG_LENGTH) {
        throw std::logic_error("Attempting to transmit canmore message larger than max length");
    }

    // Get reference to the decoder for this client, expanding the vector as necessary
    size_t prevEncoderSize = encoders.size();
    if (prevEncoderSize <= clientId) {
        encoders.resize(clientId + 1);

        // Initialize the new encoders we just made
        for (size_t i = prevEncoderSize; i < encoders.size(); i++) {
            canmore_msg_encode_init(&encoders[i], i, CANMORE_DIRECTION_AGENT_TO_CLIENT, usingCanFd());
        }
    }
    auto &encoder = encoders[clientId];

    // Load the message to encode
    canmore_msg_encode_load(&encoder, subtype, data.data(), data.size());

    // Transmit frames until done
    std::vector<uint8_t> frameBuf(getMaxFrameSize());
    do {
        uint8_t frameSize;
        uint32_t canId;
        bool isExtended;
        if (!canmore_msg_encode_next(&encoder, frameBuf.data(), &frameSize, &canId, &isExtended)) {
            throw std::runtime_error("Canmore message encoder unexpectedly failed");
        }
        assert(frameSize <= frameBuf.size());

        if (isExtended) {
            canId |= CAN_EFF_FLAG;
        }

        transmitFrame(canId, std::span { frameBuf.data(), frameSize });
    } while (!canmore_msg_encode_done(&encoder));
}

void MsgAgent::handleFrame(canid_t can_id, const std::span<const uint8_t> &data) {
    bool isExtended = !!(can_id & CAN_EFF_FLAG);
    canmore_id_t id = { .identifier = can_id };

    // Sanity check to make sure we don't have two agents running
    uint8_t direction = (isExtended ? id.pkt_ext.direction : id.pkt_std.direction);
    if (direction == CANMORE_DIRECTION_AGENT_TO_CLIENT) {
        // We've got another agent trying to talk
        // Raise an error
        handler.handleConflictingAgentError();
        return;
    }

    // Frame is from a client, begin processing
    // First determine which decoder to use
    uint8_t clientId = (isExtended ? id.pkt_ext.client_id : id.pkt_std.client_id);

    // Client ID 0 is reversed for broadcasts, don't process
    if (clientId == 0) {
        handler.handleDecodeError(clientId, CANMORE_MSG_DECODER_ERROR_INVALID_CLIENT_ID);
        return;
    }

    // Get reference to the decoder for this client, expanding the vector as necessary
    size_t prevDecoderSize = decoders.size();
    if (prevDecoderSize < clientId) {
        decoders.resize(clientId);

        // Initialize the new decoders we just made
        for (size_t i = prevDecoderSize; i < decoders.size(); i++) {
            auto &argEntry = decoderErrorArgs.emplace_back(this, i + 1);  // Client ID is Index + 1

            canmore_msg_decode_init(&decoders[i], &MsgAgent::decoderErrorCallback, &argEntry, usingCanFd());
        }
    }
    auto &decoder = decoders[clientId - 1];

    // Decode the frame
    uint32_t canIdMasked = can_id & (isExtended ? CAN_EFF_MASK : CAN_SFF_MASK);
    size_t decodeLen = canmore_msg_decode_frame(&decoder, canIdMasked, isExtended, data.data(), data.size());

    // If we got a complete message, call the receive handler
    if (decodeLen > 0) {
        uint8_t subtype = canmore_msg_decode_get_subtype(&decoder);
        uint8_t *msgBuf = canmore_msg_decode_get_buf(&decoder);
        handler.handleMessage(clientId, subtype, std::span { msgBuf, decodeLen });
    }
}
