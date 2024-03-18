#pragma once

#include "PollFD.hpp"

#include <linux/can.h>
#include <span>

namespace Canmore {

/**
 * @brief A socket wrapper for binding to CAN bus interfaces on Linux.
 * This class automatically detects if the given interface supports CAN FD and switches to FD mode (unless overidden).
 *
 * This handles all the heavy lifting of creating an interface, and exposes this socket as a PollFDHandler (see
 * PollFD.hpp for more information on what is required to process incoming packet events)
 *
 * This function can be inherited by classes which need a CAN socket. That class just needs to override the handleFrame
 * function, which is called anytime a frame is received matching the filters specified by setRxFilters. The
 * transmitFrame method can be used to transmit frames.
 *
 */
class CANSocket : public PollFDHandler {
public:
    CANSocket(int ifIndex, bool forceNoCanFd = false);
    ~CANSocket();

    // Disabling copying of CANSocket (since we have a file discriptor)
    CANSocket(CANSocket const &) = delete;
    CANSocket &operator=(CANSocket const &) = delete;

    const int ifIndex;

protected:
    /**
     * @brief Transmits the requested frame to the CAN ID
     *
     * @param can_id The ID for this CAN frame
     * @param data The data to transmit for this frame
     */
    void transmitFrame(canid_t can_id, const std::span<const uint8_t> &data);

    /**
     * @brief Transmits a frame without throwing an exception. Useful when needing to call in a destructor or siganls
     *
     * @note This function is safe to call in signals or destructors (so long as CANSocket is still valid)
     *
     * @param can_id The can id to transmit to
     * @param data The frame data to transmit
     * @return true The frame was successfully transmitted
     * @return false The frame failed to transmit
     */
    bool transmitFrameNoexcept(canid_t can_id, const uint8_t *data, size_t len) noexcept;

    /**
     * @brief Clears the socket of all pending packets
     */
    void clearRxBuffer();

    /**
     * @brief Protected function for children classes to call. This allows setting the RX Filter after the constructor.
     *
     * @param rxFilters The rx filters for the socket to receive
     */
    void setRxFilters(const std::span<can_filter> &rxFilters);

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

    /**
     * @brief Get the maximum frame size that can be transmitted by the socket
     *
     * @return size_t The max data length that can be transmitted by the CANSocket
     */
    size_t getMaxFrameSize() { return (useCanFd ? CANFD_MAX_DLEN : CAN_MAX_DLEN); }

    /**
     * @brief Reports if the socket is transmitting in CAN FD mode (detected during initialization).
     *
     * @return true The socket is in CAN FD mode (all frames transmitted as CAN FD frames)
     * @return false The socket is in standard CAN mode (all frames transmitted as standard CAN frames)
     */
    bool usingCanFd() { return useCanFd; }

private:
    int socketFd;
    bool useCanFd;
    std::shared_ptr<PollFDDescriptor> socketPollDescriptor;
};

};  // namespace Canmore
