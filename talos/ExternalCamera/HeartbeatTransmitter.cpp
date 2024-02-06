#include "DFCDaemon.hpp"

#include "titan/canmore.h"

#include <linux/can.h>
#include <linux/can/bcm.h>
#include <linux/can/raw.h>
#include <string.h>
#include <sys/socket.h>
#include <system_error>
#include <unistd.h>

HeartbeatTransmitter::HeartbeatTransmitter(int ifIndex, int clientId):
    ifIndex(ifIndex), clientId(clientId), socketFd(-1) {
    // Open socket
    // Using CAN broadcast manager so we don't need to spawn a whole thread for the heartbeat
    if ((socketFd = socket(PF_CAN, SOCK_DGRAM | SOCK_CLOEXEC, CAN_BCM)) < 0) {
        throw std::system_error(errno, std::generic_category(), "CAN BCM socket");
    }

    // Connect to the Broadcast Manager on the correct interface
    struct sockaddr_can addr = {};
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifIndex;

    if (connect(socketFd, (struct sockaddr *) &addr, sizeof(addr)) < 0) {
        close(socketFd);
        socketFd = -1;
        throw std::system_error(errno, std::generic_category(), "BCM connect");
    }
}

void HeartbeatTransmitter::start() {
    // We need a frame for each count in the heartbeat
    const static size_t num_frames = 1 << CANMORE_HEARTBEAT_CNT_LENGTH;

    // Message Struct Foramt
    struct {
        struct bcm_msg_head msg_head;
        struct can_frame frame[num_frames];
    } heartbeat_msg;

    // Configure the repeating message
    // Will transmit every heartbeat interval, looping through all the frame indices
    heartbeat_msg.msg_head.opcode = TX_SETUP;
    heartbeat_msg.msg_head.flags = SETTIMER | STARTTIMER;
    heartbeat_msg.msg_head.count = 0;
    heartbeat_msg.msg_head.ival1 = { .tv_sec = 0, .tv_usec = 0 };
    heartbeat_msg.msg_head.ival2 = { .tv_sec = HEARTBEAT_INTERVAL_MS / 1000, .tv_usec = HEARTBEAT_INTERVAL_MS * 1000 };
    heartbeat_msg.msg_head.can_id = CANMORE_CALC_UTIL_ID_C2A(clientId, CANMORE_CHAN_HEARTBEAT);
    heartbeat_msg.msg_head.nframes = num_frames;

    // Fill out heartbeat messages for each frame count
    for (size_t i = 0; i < num_frames; i++) {
        heartbeat_msg.frame[i].can_id = CANMORE_CALC_UTIL_ID_C2A(clientId, CANMORE_CHAN_HEARTBEAT);
        heartbeat_msg.frame[i].can_dlc = 1;
        // TODO: Add in error state encoding?
        heartbeat_msg.frame[i].data[0] =
            CANMORE_CALC_TITAN_HEARTBEAT_DATA(i, 0, CANMORE_TITAN_CONTROL_INTERFACE_MODE_LINUX, 0, 0);
    }

    if (write(socketFd, &heartbeat_msg, sizeof(heartbeat_msg)) < 0) {
        throw std::system_error(errno, std::generic_category(), "BCM write");
    }
}

HeartbeatTransmitter::~HeartbeatTransmitter() {
    if (socketFd >= 0)
        close(socketFd);
}
