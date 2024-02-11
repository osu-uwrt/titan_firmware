#pragma once

#include "canmore_cpp/PollFD.hpp"
#include "canmore_cpp/RegMappedClient.hpp"
#include "canmore_cpp/RemoteTTYClient.hpp"

#include <span>
#include <termios.h>

class RemoteTTYStdioHandler {
    friend class RemoteTTYStdioManager;

protected:
    // To be implemented by classes handling stdio terminal input
    virtual void handleTerminalInput(const std::span<const uint8_t> &input) = 0;
};

class RemoteTTYStdioManager : public Canmore::PollFDHandler {
public:
    RemoteTTYStdioManager(RemoteTTYStdioHandler &handler, unsigned int maxInputSize):
        handler(handler), maxInputSize(maxInputSize) {}
    ~RemoteTTYStdioManager() { cleanupTerm(); }

    void initTerm();
    void cleanupTerm() noexcept;

    void writeStdout(const std::span<const uint8_t> &output);
    void writeStderr(const std::span<const uint8_t> &output);

protected:
    // Overrides for PollFD
    void populateFds(std::vector<std::pair<PollFDHandler *, pollfd>> &fds) override;
    void handleEvent(const pollfd &fd) override;

private:
    RemoteTTYStdioHandler &handler;
    bool stdioConfigured = false;
    unsigned int maxInputSize;
    struct termios oldt;
};

class RemoteTTYClientTask : public RemoteTTYStdioHandler, Canmore::RemoteTTYClientEventHandler {
public:
    RemoteTTYClientTask(std::shared_ptr<Canmore::RegMappedCANClient> canClient);

    void run();

protected:
    // Implement the RemoteTTYStdioHandler callback
    void handleTerminalInput(const std::span<const uint8_t> &input) override;

    // Implement callbacks for RemoteTTYClient
    void handleStdout(const std::span<const uint8_t> &data) override;
    void handleStderr(const std::span<const uint8_t> &data) override;
    void handleStdinReady() override;

private:
    Canmore::RemoteTTYClient client;
    RemoteTTYStdioManager stdioManager;
    bool stopReceived = false;
};
