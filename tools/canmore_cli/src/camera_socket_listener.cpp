#include "CameraSocketListener.hpp"

#include <iostream>
#include <netdb.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <unistd.h>

#define CTRL_TYPE_STREAM_ENABLE 1
#define CTRL_TYPE_QUALITY 2
#define CTRL_TYPE_KEYPRESS 3
#define CTRL_TYPE_STREAM_SELECT 4
#define CTRL_TYPE_MAX_DIMENSION 5

class InterruptPollListener : public Canmore::PollFDHandler {
public:
    InterruptPollListener() { stdinDescriptor_ = Canmore::PollFDDescriptor::create(*this, STDIN_FILENO, POLLIN); }

    void populateFds(std::vector<std::weak_ptr<Canmore::PollFDDescriptor>> &descriptors) override {
        descriptors.push_back(stdinDescriptor_);
    }

    bool isInterrupted() { return interruptSeen_; }

protected:
    void handleEvent(const pollfd &fd) override {
        if (fd.fd != STDIN_FILENO)
            return;

        if (fd.revents & (POLLERR | POLLHUP)) {
            // Stdin closed, interrupt now
            interruptSeen_ = true;
        }
        else if (fd.revents & POLLIN) {
            char c;
            int len = read(STDIN_FILENO, &c, sizeof(c));
            if (len == 1) {
                // Got a character, see if control c
                if (c == 3) {
                    interruptSeen_ = true;
                }
            }
            else {
                // We got an error during read, just interrupt since stdin broke
                interruptSeen_ = true;
            }
        }
    }

private:
    std::shared_ptr<Canmore::PollFDDescriptor> stdinDescriptor_;
    bool interruptSeen_ = false;
};

TCPSocketServer::TCPSocketServer(CameraSocketListener &listener, int connFd, const sockaddr_in6 &sa):
    listener_(listener), sa_(sa), connFd_(connFd) {
    char name[INET6_ADDRSTRLEN];
    char port[10];
    if (getnameinfo((sockaddr *) &sa_, sizeof(sa_), name, sizeof(name), port, sizeof(port),
                    NI_NUMERICHOST | NI_NUMERICSERV) == 0) {
        std::cout << "(" << connFd << ") Accepted new client from [" << name << "]:" << port << std::endl;
    }
    else {
        std::cout << "(" << connFd << ") Accepted new client from ????:??" << std::endl;
    }

    // Register to receive any incoming data on this socket
    fdDescriptor_ = Canmore::PollFDDescriptor::create(*this, connFd_, POLLIN, true);
}

TCPSocketServer::~TCPSocketServer() {
    if (connFd_ != -1) {
        close(connFd_);
        if (alive_) {
            std::cout << "(" << connFd_ << ") Connection closed due to class destruction" << std::endl;
        }
    }
}

void TCPSocketServer::populateFds(std::vector<std::weak_ptr<Canmore::PollFDDescriptor>> &descriptors) {
    descriptors.push_back(fdDescriptor_);
}

void TCPSocketServer::transmitJpeg(const std::span<const uint8_t> &data) {
    if (data.size() > UINT32_MAX) {
        throw std::runtime_error("Attempting to transmit JPEG larger than maximum write size");
    }
    uint32_t dataSize = (uint32_t) data.size();

    // Send the header
    uint8_t header[8];
    header[0] = 0xA5;
    header[1] = 0x5A;
    header[2] = 0x0F;
    header[3] = 0xF0;
    header[4] = (dataSize >> 24) & 0xFF;
    header[5] = (dataSize >> 16) & 0xFF;
    header[6] = (dataSize >> 8) & 0xFF;
    header[7] = dataSize & 0xFF;
    if (write(connFd_, header, sizeof(header)) != sizeof(header)) {
        std::cout << "(" << connFd_ << ") Failed to write: " << errno << " - Closing connection..." << std::endl;
        alive_ = false;
        listener_.reportChildServerDeath();
        return;
    }

    // Keep looping until we get all the bytes written
    auto itr = data.begin();
    while (itr != data.end()) {
        size_t remainingLen = data.end() - itr;

        // Transmit any bytes as possible
        int len = write(connFd_, &(*itr), remainingLen);
        if (len < 0) {
            std::cout << "(" << connFd_ << ") Failed to write: " << errno << " - Closing connection..." << std::endl;
            alive_ = false;
            listener_.reportChildServerDeath();
            return;
        }
        else if (len == 0) {
            std::cout << "(" << connFd_ << ") Socket did not write any bytes? - Closing connection..." << std::endl;
            alive_ = false;
            listener_.reportChildServerDeath();
            return;
        }

        itr += len;
    }
}

void TCPSocketServer::handleEvent(const pollfd &fd) {
    if (fd.fd != connFd_) {
        throw std::logic_error("Event called for invalid conn fd");
    }

    if (fd.revents & POLLIN) {
        int curlen = pendingRxBuf_.size();
        int rxlen;
        if (curlen < 2) {
            // Need to receive the header if we haven't received it yet
            pendingRxBuf_.resize(2);
            rxlen = read(connFd_, pendingRxBuf_.data() + curlen, pendingRxBuf_.size() - curlen);
        }
        else {
            // Receive the rest of the packet based on the length
            pendingRxBuf_.resize(2 + pendingRxBuf_.at(1));
            rxlen = read(connFd_, pendingRxBuf_.data() + curlen, pendingRxBuf_.size() - curlen);
        }

        // Hanle errors during receive
        if (rxlen == 0) {
            std::cout << "(" << connFd_ << ") Connection closed" << std::endl;
            alive_ = false;
            listener_.reportChildServerDeath();
            return;
        }
        else if (rxlen < 0) {
            std::cout << "(" << connFd_ << ") Failed to read: " << errno << " - Closing connection..." << std::endl;
            alive_ = false;
            listener_.reportChildServerDeath();
            return;
        }

        // We got a packet, resize the pending buffer to that length
        pendingRxBuf_.resize(curlen + rxlen);

        // Check if we have a full packet
        size_t len = pendingRxBuf_.size();
        if (len >= 2 && len == pendingRxBuf_.at(1) + 2u) {
            uint8_t cmd = pendingRxBuf_.at(0);
            uint8_t param_len = pendingRxBuf_.at(1);
            switch (cmd) {
            case CTRL_TYPE_STREAM_ENABLE:
                if (param_len == 0) {
                    listener_.imageRx_.setStreamEnabled(true);
                }
                break;
            case CTRL_TYPE_KEYPRESS:
                listener_.imageRx_.sendKeypress(pendingRxBuf_.at(2));
                break;
            case CTRL_TYPE_QUALITY:
                if (param_len == 1) {
                    listener_.imageRx_.setStreamQuality(pendingRxBuf_.at(2));
                }
                break;
            case CTRL_TYPE_STREAM_SELECT:
                if (param_len == 1) {
                    listener_.imageRx_.setStreamId(pendingRxBuf_.at(2));
                }
                break;
            case CTRL_TYPE_MAX_DIMENSION:
                if (param_len == 2) {
                    uint16_t maxDim = (((uint16_t) pendingRxBuf_.at(2)) << 8) | pendingRxBuf_.at(3);
                    listener_.imageRx_.setMaxDimension(maxDim);
                }
                break;
            }
            pendingRxBuf_.clear();
        }
    }
    else if (fd.revents) {
        std::cout << "(" << connFd_ << ") Connection closed";
        alive_ = false;
        listener_.reportChildServerDeath();
    }
}

CameraSocketListener::CameraSocketListener(uint16_t port, int canIfIndex, uint8_t canClientId, bool loopbackOnly):
    imageRx_(canIfIndex, canClientId, *this) {
    sockFd_ = socket(AF_INET6, SOCK_STREAM, IPPROTO_TCP);
    if (sockFd_ <= 0) {
        throw std::system_error(errno, std::generic_category(), "socket(AF_INET6, SOCK_STREAM, IPPROTO_TCP)");
    }

    // Allow address reuse
    int flag = 1;
    if (setsockopt(sockFd_, SOL_SOCKET, SO_REUSEADDR, &flag, sizeof(flag))) {
        close(sockFd_);
        throw std::system_error(errno, std::generic_category(), "setsockopt(SO_REUSEADDR, 1)");
    }

    // Allow both IPv4 and IPv6 on socket
    flag = 0;
    if (setsockopt(sockFd_, IPPROTO_IPV6, IPV6_V6ONLY, &flag, sizeof(flag))) {
        close(sockFd_);
        throw std::system_error(errno, std::generic_category(), "setsockopt(IPV6_V6ONLY, 0)");
    }

    // Bind to the socket
    struct sockaddr_in6 serverAddr = {};
    serverAddr.sin6_family = AF_INET6;
    if (loopbackOnly) {
        serverAddr.sin6_addr = IN6ADDR_LOOPBACK_INIT;
    }
    else {
        serverAddr.sin6_addr = IN6ADDR_ANY_INIT;
    }
    serverAddr.sin6_port = htons(port);

    // Bind to the address
    if (bind(sockFd_, (struct sockaddr *) &serverAddr, sizeof(serverAddr))) {
        close(sockFd_);
        throw std::system_error(errno, std::generic_category(), "bind(...)");
    }

    // Start listening on socket
    if (listen(sockFd_, LISTEN_BACKLOG)) {
        close(sockFd_);
        throw std::system_error(errno, std::generic_category(), "listen(...)");
    }

    // Popoulate the poll fd descriptor to be notified of incoming events
    pollFdDescriptor_ = Canmore::PollFDDescriptor::create(*this, sockFd_, POLLIN, true);
    pollGroup_.addFd(*this);

    // Set the last added server iterator to valid value
    nextServerToAdd_ = servers_.end();
}

CameraSocketListener::~CameraSocketListener() {
    close(sockFd_);
}

void CameraSocketListener::run() {
    InterruptPollListener interruptListener;
    pollGroup_.addFd(interruptListener);

    while (!interruptListener.isInterrupted()) {
        // Run the poll group
        pollGroup_.processEvent(1000);

        // Handle any newly added/removed servers
        processPollGroupUpdates();
    }
}

void CameraSocketListener::populateFds(std::vector<std::weak_ptr<Canmore::PollFDDescriptor>> &descriptors) {
    descriptors.push_back(pollFdDescriptor_);
    imageRx_.populateFds(descriptors);
}

void CameraSocketListener::handleEvent(const pollfd &fd) {
    if (fd.fd != sockFd_) {
        throw std::logic_error("Event called for invalid conn fd");
    }

    if (fd.revents & (POLLERR | POLLHUP)) {
        throw std::runtime_error("Listener socket entered error state");
    }

    if (fd.revents & POLLIN) {
        sockaddr_in6 clientAddr;
        socklen_t clientAddrLen = sizeof(clientAddr);
        int connFd = accept(sockFd_, (struct sockaddr *) &clientAddr, &clientAddrLen);
        if (connFd == -1) {
            // Some sort of error occurred, this could be that the client aborted the connection, just silently drop
            return;
        }

        if (servers_.empty()) {
            // If this is the first client to connect, send the enable request
            imageRx_.setStreamEnabled(true);
        }

        // Add server processor to this class
        servers_.emplace_back(*this, connFd, clientAddr);
        if (nextServerToAdd_ == servers_.end()) {
            nextServerToAdd_ = std::prev(servers_.end());
        }
    }
}

void CameraSocketListener::handleJpeg(const std::span<const uint8_t> &data) {
    if (servers_.empty()) {
        // We don't have any listening servers, but we got a frame?
        // Disable the stream (we must have lost the first packet)
        imageRx_.setStreamEnabled(false);
        return;
    }

    for (auto &server : servers_) {
        server.transmitJpeg(data);
    }
}

void CameraSocketListener::processPollGroupUpdates() {
    // Handle any new servers added
    while (nextServerToAdd_ != servers_.end()) {
        pollGroup_.addFd(*nextServerToAdd_);
        nextServerToAdd_++;
    }

    // Handle any new servers that died
    if (serverHadDeath_) {
        auto itr = servers_.begin();
        while (itr != servers_.end()) {
            if (!itr->getAlive()) {
                itr = servers_.erase(itr);
            }
            else {
                itr++;
            }
        }
        serverHadDeath_ = false;

        if (servers_.empty()) {
            // If we removed the last server, disable CAN bus stream to reduce bandwidth
            imageRx_.setStreamEnabled(false);
        }

        // Fix the iterator after erasing stuff (we know its okay since we added everything before this)
        nextServerToAdd_ = servers_.end();
    }
}
