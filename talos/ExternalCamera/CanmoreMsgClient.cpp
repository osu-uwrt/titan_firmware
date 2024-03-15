#include "CanmoreMsgClient.hpp"

using namespace Canmore;

MsgClient::MsgClient(int ifIndex, uint8_t clientId, MsgHandler &handler):
    CANSocket(ifIndex), clientId(clientId), handler(handler) {
    // Check if the socket initialized in CAN FD mode - configure the encoder/decoder with this
    bool useFd = usingCanFd();

    // Initialize Encoder/Decoder
    canmore_msg_encode_init(&encoder, clientId, CANMORE_DIRECTION_CLIENT_TO_AGENT, useFd);
    canmore_msg_decode_init(&decoder, &MsgClient::decoderErrorCallback, this, useFd);

    // Setup Receive Filter
    // Need to match both standard CAN frames and extended CAN message frames for this client from the agent
    struct can_filter rfilter[] = {
        { .can_id = CANMORE_CALC_MSG_ID_A2C(clientId, 0),
          .can_mask = (CAN_EFF_FLAG | CAN_RTR_FLAG | CANMORE_CALC_FILTER_MASK(1, 1, 1, 0)) },
        { .can_id = CAN_EFF_FLAG | CANMORE_CALC_MSG_EXT_ID_A2C(clientId, 0, 0),
          .can_mask = (CAN_EFF_FLAG | CAN_RTR_FLAG | CANMORE_CALC_EXT_FILTER_MASK(1, 1, 1, 0, 0)) },
    };
    setRxFilters(std::span { rfilter });
}

void MsgClient::transmitMessage(uint8_t subtype, std::span<uint8_t> data) {
    if (subtype >= (1 << CANMORE_MSG_SUBTYPE_LENGTH)) {
        throw std::logic_error("Attempting to transmit canmore message with invalid message subtype");
    }
    if (data.size() > CANMORE_MAX_MSG_LENGTH) {
        throw std::logic_error("Attempting to transmit canmore message larger than max length");
    }
    canmore_msg_encode_load(&encoder, subtype, data.data(), data.size());

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
    } while ((!canmore_msg_encode_done(&encoder)));
}

void MsgClient::handleFrame(canid_t canId, const std::span<const uint8_t> &data) {
    bool isExtended = (canId & CAN_EFF_FLAG ? true : false);
    uint32_t canIdMasked = canId & (isExtended ? CAN_EFF_MASK : CAN_SFF_MASK);

    // Decode the frame
    size_t decodeLen = canmore_msg_decode_frame(&decoder, canIdMasked, isExtended, data.data(), data.size());

    // If we got a complete message, call the receive handler
    if (decodeLen > 0) {
        uint8_t subtype = canmore_msg_decode_get_subtype(&decoder);
        uint8_t *msgBuf = canmore_msg_decode_get_buf(&decoder);
        handler.handleMessage(subtype, std::span { msgBuf, decodeLen });
    }
}
