#pragma once
#include "canmore_cpp/CANSocket.hpp"

#include <vector>

namespace Canmore {

class ImageReceiverHandler {
protected:
    friend class ImageReceiver;
    virtual void handleJpeg(const std::span<const uint8_t> &data) = 0;
};

class ImageReceiver : public CANSocket {
public:
    /**
     * @brief Construct a new Canmore Image Receiver object
     *
     * @param ifIndex The interface index to bind to
     * @param clientId The client ID this image transmitter belongs to
     * @param handler The handler to call with the JPEG upon successful reception
     */
    ImageReceiver(int ifIndex, uint8_t clientId, ImageReceiverHandler &handler);
    ~ImageReceiver();

    /**
     * @brief Enables/Disables the remote stream
     *
     * @param enable Set to true to enable the remote stream, false to disable
     */
    void setStreamEnabled(bool enable);

    /**
     * @brief Requests the given stream id from the client
     *
     * @param streamId The stream id to request (defined by application)
     */
    void setStreamId(uint8_t streamId);

    /**
     * @brief Sets the stream quality for the requested stream
     *
     * @param quality The JPEG compression quality (must be 0-100)
     */
    void setStreamQuality(uint8_t quality);

    /**
     * @brief Sends a keypress event
     *
     * @param keypress The keypress character to send
     */
    void sendKeypress(uint8_t keypress);

    /**
     * @brief Sets the max image dimension to transmit over CAN bus
     *
     * @param maxDim The max image dimension
     */
    void setMaxDimension(uint16_t maxDim);

    const uint8_t clientId;

protected:
    void handleFrame(canid_t can_id, const std::span<const uint8_t> &data) override;

private:
    ImageReceiverHandler &handler_;
    bool decodeStop_ = true;
    std::vector<uint8_t> rxBuf_;
    uint32_t nextFrameIdx_;
};

};  // namespace Canmore
