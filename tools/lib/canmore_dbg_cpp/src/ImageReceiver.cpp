#include "canmore_cpp/ImageReceiver.hpp"

#include "canmore/camera_feed_interface.h"
#include "canmore/crc32.h"
#include "canmore/protocol.h"

using namespace Canmore;

ImageReceiver::ImageReceiver(int ifIndex, uint8_t clientId, ImageReceiverHandler &handler):
    Canmore::CANSocket(ifIndex), clientId(clientId), handler_(handler) {
    // Register to receive stream frames and the last frame
    struct can_filter rfilter[] = { { .can_id = CANMORE_CAMERA_FEED_CALC_LAST_FRAME_ID(clientId),
                                      .can_mask = (CAN_EFF_FLAG | CAN_RTR_FLAG | CANMORE_CAMERA_FEED_STD_ID_MASK) },
                                    { .can_id = CAN_EFF_FLAG | CANMORE_CAMERA_FEED_CALC_FRAME_ID(clientId, 0),
                                      .can_mask = (CAN_EFF_FLAG | CAN_RTR_FLAG | CANMORE_CAMERA_FEED_EXT_ID_MASK) } };
    setRxFilters(std::span { rfilter });
}

void ImageReceiver::handleFrame(canid_t can_id, const std::span<const uint8_t> &data) {
    bool isExtended = (can_id & CAN_EFF_FLAG) != 0;
    canmore_id_t id = { .identifier = can_id };

    if (isExtended) {
        // Standard stream frame, process it
        if (id.pkt_ext.extra == 0) {
            // First packet, reset decoder state
            nextFrameIdx_ = 1;
            rxBuf_.resize(0);
            decodeStop_ = false;
        }
        else {
            if (decodeStop_) {
                // We are told to stop decoding, dont' do anything else
                return;
            }
            if (id.pkt_ext.extra != nextFrameIdx_) {
                // Sequence didn't line up
                decodeStop_ = true;
                return;
            }
            nextFrameIdx_++;
        }

        // Add the received frame to the receive buffer
        rxBuf_.insert(rxBuf_.end(), data.begin(), data.end());
    }
    else {
        // This is a standard frame, it should be the last one in the packet
        if (decodeStop_) {
            // Don't do any additional processing if decode told to stop
            return;
        }
        // Whether we pass or not, this should be the last frame until the next one, just set the stop flag now
        decodeStop_ = true;

        canmore_camera_feed_last_frame_t lastFrame;
        if (data.size() != sizeof(lastFrame)) {
            // Invalid last frame size, fail the decode
            return;
        }
        // Copy in data to last frame
        std::copy(data.begin(), data.end(), lastFrame.data);

        // Trim rx buffer to the requested length
        if (rxBuf_.size() < lastFrame.pkt.len) {
            // Length longer than received packet
            return;
        }
        else if (rxBuf_.size() > lastFrame.pkt.len) {
            // Resize the receive buffer to its real length
            rxBuf_.resize(lastFrame.pkt.len);
        }

        // Verify the crc32
        uint32_t crcComputed = crc32_compute(rxBuf_.data(), rxBuf_.size());
        if (crcComputed != lastFrame.pkt.crc32) {
            // Checksum didn't match
            return;
        }

        // Packet checks out okay, good to report to handler
        handler_.handleJpeg(rxBuf_);
    }
}

void ImageReceiver::setStreamEnabled(bool enable) {
    canmore_camera_feed_cmd_t cmd = { .pkt = { .cmd = CANMORE_CAMERA_FEED_CMD_ENABLE, .data = { .enable = enable } } };
    transmitFrame(CANMORE_CAMERA_FEED_CALC_CTRL_ID(clientId), cmd.data);
}

void ImageReceiver::setStreamId(uint8_t streamId) {
    canmore_camera_feed_cmd_t cmd = { .pkt = { .cmd = CANMORE_CAMERA_FEED_CMD_STREAM_ID,
                                               .data = { .stream_id = streamId } } };
    transmitFrame(CANMORE_CAMERA_FEED_CALC_CTRL_ID(clientId), cmd.data);
}

void ImageReceiver::setStreamQuality(uint8_t quality) {
    canmore_camera_feed_cmd_t cmd = { .pkt = { .cmd = CANMORE_CAMERA_FEED_CMD_QUALITY,
                                               .data = { .quality = quality } } };
    transmitFrame(CANMORE_CAMERA_FEED_CALC_CTRL_ID(clientId), cmd.data);
}
