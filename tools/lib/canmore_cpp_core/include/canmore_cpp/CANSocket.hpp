#pragma once

#include "PollFD.hpp"

#include <linux/can.h>
#include <span>

namespace Canmore {

class CANSocket : public PollFDHandler {
public:
    CANSocket(int ifIndex);
    CANSocket(int ifIndex, const std::span<can_filter> &&rxFilters);
    ~CANSocket();

    // Disabling copying of CANSocket (since we have a file discriptor)
    CANSocket(CANSocket const &) = delete;
    CANSocket &operator=(CANSocket const &) = delete;

    const int ifIndex;

protected:
    void transmitFrame(canid_t can_id, const std::span<const uint8_t> &data);
    void clearRxBuffer();

    /**
     * @brief Protected function for children classes to call. This allows setting the RX Filter after the constructor.
     *
     * @param rxFilters The rx filters for the socket to receive
     */
    void setRxFilters(const std::span<can_filter> &rxFilters) { setRxFiltersInternal(rxFilters, false); }

    /**
     * @brief Handle Frame method to be implemented by children.
     *
     * This method is called a new CAN frame is received
     *
     * @param can_id The CAN ID of the received frame
     * @param data The data in the received frame
     */
    virtual void handleFrame(canid_t can_id, const std::span<const uint8_t> &data) = 0;

    /*
     * Overrides for PollFD - Implemented by this class
     */
    void populateFds(std::vector<std::weak_ptr<PollFDDescriptor>> &descriptors) override;
    void handleEvent(const pollfd &fd) override;

private:
    // Separate internal function so the constructor can request that the socket be closed when an exception is thrown
    void setRxFiltersInternal(const std::span<can_filter> &rxFilters, bool closeOnFail);
    int socketFd;
    std::shared_ptr<PollFDDescriptor> socketPollDescriptor;
};

};  // namespace Canmore
