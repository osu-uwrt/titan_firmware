#pragma once

#include "titan/canmore/remote_tty_interface.h"

#include <chrono>
#include <cstdint>
#include <deque>
#include <span>
#include <stdexcept>
#include <vector>

namespace Canmore {

class RemoteTTYStreamTXCallback {
    friend class RemoteTTYStreamTXScheduler;

protected:
    /**
     * @brief Function called by RemoteTTYStreamTXScheduler whenever a packet needs to be transmitted.
     *
     * This is called whenever a new packet is written, as well as whenever the receiver requests a retransmit.
     *
     * @param streamId The stream id for the packet (whatever is passed to transmitterWrite)
     * @param seqNum The sequence number for the packet (as defined in the remote tty interface)
     * @param data The data for the packet
     */
    virtual void transmitStreamPacket(uint8_t streamId, uint16_t seqNum, const std::span<const uint8_t> &data) = 0;

    /**
     * @brief Notify that space has been freed in the unacked buffer
     */
    virtual void notifyBufferAvailable() = 0;
};

class RemoteTTYStreamRXCallback {
    friend class RemoteTTYStreamRXScheduler;

protected:
    /**
     * @brief Function called by RemoteTTYStreamRXScheduler whenever an ack packet needs to be sent
     *
     * @param seqNum The sequence number being acknowledged (as defined in the remote tty interface)
     */
    virtual void transmitAck(uint16_t seqNum) = 0;
};

class RemoteTTYStreamTXScheduler {
public:
    /**
     * @brief Construct a new Remote TTY Stream Transmit Scheduler.
     *
     * This performs all of the complex packet re-transmission and state recovery
     *
     * @param callback The callback interface to call when transmitting packets
     * @param maxInFlight The maximum number of unacknowledged packets that can be in flight at once
     */
    RemoteTTYStreamTXScheduler(RemoteTTYStreamTXCallback &callback,
                               uint16_t maxInFlight = CANMORE_REMOTE_TTY_DEFAULT_MAX_IN_FLIGHT):
        maxInFlight(maxInFlight),
        callback(callback) {}

    /**
     * @brief Checks if the transmit buffer has space available
     *
     * @return true If there is space for another packet to be written
     * @return false No more space is available for in flight packets. transmitterWrite should not be called
     */
    bool spaceAvailable() { return unackedBuffer.size() < maxInFlight; }

    /**
     * @brief Transmits the requested packet on the given stream
     *
     * @attention transmitterSpaceAvailable() must return true before calling this function
     *
     * @param streamId The stream ID for this packet (passed to transmitPacket)
     * @param data The data to transmit
     */
    void write(uint8_t streamId, const std::span<const uint8_t> &data);

    /**
     * @brief Notifies that an ack has been received from the receiver.
     *
     * This handles all of the processing as required by Canmore to either advanced the unacked buffer, retransmit
     * packets, or reset the connection state.
     *
     * @param seqNum The sequence number that was acked
     */
    void notifyAck(uint16_t seqNum);

    /**
     * @brief The maximum number of unacked packets that can be in flight. Set during construction
     */
    const uint16_t maxInFlight;

private:
    RemoteTTYStreamTXCallback &callback;
    uint16_t lastAckedSeqNum = 0;
    std::deque<std::pair<uint8_t, std::vector<uint8_t>>> unackedBuffer;

    /**
     * @brief Compute the given packets sequence number for a given index in the unackedBuffer
     *
     * @param itr Iterator to unackedBuffer
     * @return uint16_t The sequence number for that packet
     */
    uint16_t computeSeqNum(std::deque<std::pair<uint8_t, std::vector<uint8_t>>>::const_iterator itr) {
        size_t bufferIdx = itr - unackedBuffer.begin();

        // Typically, the sequence number is the buffer index added to the sequence number for the start of the buffer
        // The front of the deque buffer's index is given by lastAckedSeqNum + 1
        uint16_t newSeqNum = bufferIdx + lastAckedSeqNum + 1;

        // If we roll over, we must add 1 since 0 isn't a valid index, so it must be skipped
        if (newSeqNum < lastAckedSeqNum)
            newSeqNum++;

        return newSeqNum;
    }
};

class RemoteTTYStreamRXScheduler {
public:
    /**
     * @brief Construct a new Remote TTY Stream Receive Scheduler
     *
     * This class verifies if a given sequence number is the next expected packet in the sequence. It will also
     * schedule acknowledgement packets to be sent out after a given number of packets are received, or enough time
     * elapses.
     *
     * This does the heavy lifting of the Remote TTY stream receiver, as defined in the Canmore protocol.
     *
     * @param callback Callback for transmitting acks
     * @param maxBeforeAck Transmits an ack when this many packets have been successfully received since an ack has last
     * been sent. This should ideally be less than the transmitters maxInFlight value.
     * @param ackTransmitIntervalMs The time in ms since a packet was received before an ack is sent out
     */
    RemoteTTYStreamRXScheduler(RemoteTTYStreamRXCallback &callback,
                               uint16_t maxBeforeAck = CANMORE_REMOTE_TTY_DEFAULT_RX_PACKETS_PER_ACK,
                               unsigned long ackTransmitIntervalMs = CANMORE_REMOTE_TTY_DEFAULT_ACK_INTERVAL_MS):
        maxBeforeAck(maxBeforeAck),
        ackTransmitInterval(ackTransmitIntervalMs), callback(callback) {}

    /**
     * @brief Checks if the given sequence number is the next expected packet in the Remote TTY sequence.
     *
     * This also counts the packet as "received" if this function returns true. After maxBeforeAck successfully received
     * packets, this function will request that an ack be sent.
     *
     * @param seqNum The sequence number for the received packet
     * @return true The packet is the next in the sequence. The receiver should process this packet
     * @return false The packet is not the next in the sequence. The receiver should discard this packet
     */
    bool checkPacket(uint16_t seqNum);

    /**
     * @brief Reports that the timer has fired, and does any background required by the timer (such as sending a
     * periodic ack).
     *
     * @attention This function must be called immediately after construction to send the first ack, notifying the
     * transmitter it is okay to begin transmitting data.
     *
     * @details This function is required as the Remote TTY protocol requires the receiver to periodically send an
     * ack if no packet was received over a given interval (controlled by ackTransmitInterval). It is left up to the
     * caller of this class to determine what mechanism this occurs (but note this class is not thead or signal safe).
     *
     * The recommended method is by using a timerfd, which integrates well with the poll() calls used by the CAN socket.
     * Calling this function will check the internal state, and send an ack packet if required. The number of ms before
     * this function should be called again is returned. This duration can then be used to set whatever timer is used
     * to schedule the next time receiverHandleTimer() should be called
     *
     * You do not need to worry about if a packet was received between when this function was scheduled, as this takes
     * it into account. If an ack does not need to be sent when this is called again (such as due to a packet being
     * received since this function was last called) an updated duration will be returned, taking into account this
     * last receive time.
     *
     * @return unsigned long The duration in ms for when this function should be called again
     */
    unsigned long handleTimer();

    /**
     * @brief Transmits an ack when this many packets have been successfully received since an ack has last been sent
     */
    const uint16_t maxBeforeAck;

    /**
     * @brief The interval in milliseconds since the last packet was received before an ack will be sent.
     */
    const std::chrono::milliseconds ackTransmitInterval;

private:
    RemoteTTYStreamRXCallback &callback;
    uint16_t unackedPackets = 0;
    uint16_t lastReceivedSeqNum = 0;

    /**
     * @brief Time which the next time scheduled ack should be sent
     *
     * Set the next scheduled ack to now, to force the scheduler to immediately transmit a 0 as required by the
     * protocol (this resets the transmitter's state)
     */
    std::chrono::time_point<std::chrono::steady_clock> nextScheduledAck = std::chrono::steady_clock::now();

    /**
     * @brief Updates nextScheduledAck to the current time + ackTransmitIntervalMs
     *
     * Should be called whenever a new packet is successfully received.
     */
    void updateNextScheduledAck() { nextScheduledAck = std::chrono::steady_clock::now() + ackTransmitInterval; }
};

};  // namespace Canmore
