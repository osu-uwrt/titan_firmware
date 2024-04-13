#pragma once

#include "canmore_cpp/ImageReceiver.hpp"
#include "canmore_cpp/PollFD.hpp"

#include <list>
#include <netinet/in.h>

class CameraSocketListener;

class TCPSocketServer : public Canmore::PollFDHandler {
public:
    TCPSocketServer(CameraSocketListener &listener, int connFd, const sockaddr_in6 &sa);
    ~TCPSocketServer();

    void transmitJpeg(const std::span<const uint8_t> &data);
    bool getAlive() { return alive_; }

protected:
    void populateFds(std::vector<std::weak_ptr<Canmore::PollFDDescriptor>> &descriptors) override;
    void handleEvent(const pollfd &fd) override;

private:
    CameraSocketListener &listener_;
    bool alive_ = true;
    std::shared_ptr<Canmore::PollFDDescriptor> fdDescriptor_;
    sockaddr_in6 sa_;
    const int connFd_;
};

class CameraSocketListener : protected Canmore::ImageReceiverHandler, protected Canmore::PollFDHandler {
    friend class TCPSocketServer;

public:
    CameraSocketListener(uint16_t port, int canIfIndex, uint8_t canClientId, bool loopbackOnly = false);

    void run();

protected:
    void handleJpeg(const std::span<const uint8_t> &data) override;

    void populateFds(std::vector<std::weak_ptr<Canmore::PollFDDescriptor>> &descriptors) override;
    void handleEvent(const pollfd &fd) override;

    void reportChildServerDeath() { serverHadDeath_ = true; }

    Canmore::ImageReceiver imageRx_;

private:
    void processPollGroupUpdates();

    Canmore::PollGroup pollGroup_;
    int sockFd_;
    bool serverHadDeath_ = false;
    std::list<TCPSocketServer> servers_;
    std::list<TCPSocketServer>::iterator nextServerToAdd_;
    std::shared_ptr<Canmore::PollFDDescriptor> pollFdDescriptor_;

    const int LISTEN_BACKLOG = 10;
};
