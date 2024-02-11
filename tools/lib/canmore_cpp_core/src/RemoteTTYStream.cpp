#include "canmore_cpp/RemoteTTYStream.hpp"

#include <iostream>  // TODO: Remove me after validated

using namespace Canmore;

void RemoteTTYStreamTXScheduler::transmitterWrite(uint8_t streamId, const std::span<const uint8_t> &data) {
    if (!transmitterSpaceAvailable())
        throw std::logic_error("Cannot write to full TTY transmit buffer");

    // Store the packet locally so it can be retransmitted if required
    auto &newEntry = unackedBuffer.emplace_back(streamId, data.size());
    newEntry.second.assign(data.begin(), data.end());

    // Compute the sequence number for the packet we just added
    uint16_t seqNum = computeSeqNum(std::prev(unackedBuffer.end()));

    // Transmit the packet
    callback.transmitStreamPacket(streamId, seqNum, data);
}

void RemoteTTYStreamTXScheduler::transmitterNotifyAck(uint16_t seqNum) {
    if (seqNum == 0) {
        // Sequence number 0 is never a valid packet, always just reset state
        lastAckedSeqNum = 0;
        unackedBuffer.clear();

        callback.notifyBufferAvailable();
    }

    // If we get the same ack twice, that means the receiver requested a retransmission
    else if (seqNum == lastAckedSeqNum) {
        // Retransmit the entire unacked buffer again
        for (auto itr = unackedBuffer.begin(); itr != unackedBuffer.end(); itr++) {
            // Compute the packet's sequence number and retransmit
            uint16_t seqNum = computeSeqNum(itr);
            callback.transmitStreamPacket(itr->first, seqNum, itr->second);
        }
    }

    // This is just a normal packet ack
    else {
        // Figure out what index the acked sequence number is in unackedBuffer
        // We need to subtract the base index (lastAckedSeqNum + 1) from the sequence num to get the index
        uint16_t ackedIdx = seqNum - lastAckedSeqNum - 1;

        // Handle integer rollover
        // Because 0 is an invalid value, we need to subtract one to remove 0 from the modular arithmatic
        if (ackedIdx >= seqNum) {
            ackedIdx--;
        }

        // If the index isn't valid, then a packet was acked which isn't in the unackedBuffer
        // This shouldn't be possible, as we only remove packets after they are acked
        // To recover the connection, just say that's the last acked packet and clear our local buffer
        if (ackedIdx >= unackedBuffer.size()) {
            std::cerr << "Receiver lost sync with transmitter (THIS SHOULDN'T HAPPEN) - resetting buffer state"
                      << std::endl;  // TODO: Remove once validated

            unackedBuffer.clear();
        }
        else {
            // Compute an iterator for the next unacked element in the buffer
            auto nextUnackedItr = std::next(ackedIdx + unackedBuffer.begin());

            // Erase everything up to next unacked packet
            unackedBuffer.erase(unackedBuffer.begin(), nextUnackedItr);
        }

        // Mark the last acked sequence number
        lastAckedSeqNum = seqNum;

        // Finally notify that buffer space is available
        callback.notifyBufferAvailable();
    }
}

bool RemoteTTYStreamRXScheduler::receiverCheckPacket(uint16_t seqNum) {
    uint16_t expectedSeqNum = lastReceivedSeqNum + 1;
    if (expectedSeqNum == 0) {
        // On overflow, 0 isn't a valid sequence number, add another to get it to 1
        expectedSeqNum++;
    }

    // If the sequence number doesn't match what we expect, drop the packet
    if (seqNum != expectedSeqNum) {
        return false;
    }

    // Update the last received sequence number
    lastReceivedSeqNum = seqNum;

    // Increment the number of unacked packets
    unackedPackets++;
    if (unackedPackets >= maxBeforeAck) {
        unackedPackets = 0;
        callback.transmitAck(seqNum);
    }

    // Finally mark that we received a packet, delaying the periodic ack transmission to since this packet was received
    updateNextScheduledAck();

    // Say that it's okay for the packet to be processed
    return true;
}

unsigned long RemoteTTYStreamRXScheduler::receiverHandleTimer() {
    // Next scheduled ack is in the past, send an ack
    auto now = std::chrono::steady_clock::now();
    if (nextScheduledAck <= now) {
        callback.transmitAck(lastReceivedSeqNum);
        updateNextScheduledAck();
        unackedPackets = 0;

        // Tell the timer to check back in within 1 interval
        return ackTransmitInterval.count();
    }
    else {
        // We must have received a packet since the timer was scheduled
        // Return the number of milliseconds until we need to send the ack
        auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(nextScheduledAck - now);
        // If ms.count is zero, it'll actually cancel the timer. Just kick it over the edge to come back in 1 ms

        return (ms.count() > 0 ? ms.count() : 1);
    }
}
