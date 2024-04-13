#pragma once

#include "canmore_cpp/LinuxClient.hpp"
#include "canmore_cpp/PollFD.hpp"
#include "canmore_cpp/RegMappedClient.hpp"
#include "canmore_cpp/RemoteTTYClient.hpp"
#include "canmore_cpp/span_compat.hpp"

#include <signal.h>
#include <termios.h>

class RemoteTTYStdioHandler {
    friend class RemoteTTYStdioManager;

protected:
    // To be implemented by classes handling stdio terminal input
    virtual void handleTerminalInput(const std::span<const uint8_t> &input) = 0;
    virtual void handleTerminalResize(uint16_t newRows, uint16_t newCols) = 0;
    virtual bool checkStdinReady() = 0;
    virtual void handleStdinNotifyRequest() = 0;
};

class RemoteTTYStdioManager : public Canmore::PollFDHandler {
public:
    RemoteTTYStdioManager(RemoteTTYStdioHandler &handler, unsigned int maxInputSize);
    ~RemoteTTYStdioManager();

    void initTerm();
    void cleanupTerm() noexcept;

    void writeStdout(const std::span<const uint8_t> &output);
    void writeStderr(const std::span<const uint8_t> &output);
    void notifyStdinReady();

protected:
    // Overrides for PollFD
    void populateFds(std::vector<std::weak_ptr<Canmore::PollFDDescriptor>> &descriptors) override;
    void handleEvent(const pollfd &fd) override;

private:
    RemoteTTYStdioHandler &handler_;
    int winchEventFd_ = -1;
    bool stdioConfigured_ = false;
    unsigned int maxInputSize_;
    struct termios oldt_;
    RemoteTTYStdioManager *oldWinchReceiver_;
    struct sigaction oldWinchAction_;
    std::shared_ptr<Canmore::PollFDDescriptor> stdinPollDescriptor_;
    std::shared_ptr<Canmore::PollFDDescriptor> winchEventPollDescriptor_;

    void winchHandler();
    static void winchHandlerStatic(int sig);
    static RemoteTTYStdioManager *winchReceiver;
};

class RemoteTTYClientTask : public RemoteTTYStdioHandler, Canmore::RemoteTTYClientEventHandler {
public:
    RemoteTTYClientTask(std::shared_ptr<Canmore::LinuxClient> linuxRegClient,
                        std::shared_ptr<Canmore::RegMappedCANClient> canClient, const std::string &cmd,
                        bool useUploadWorkingDir);

    void run();

protected:
    // Implement the RemoteTTYStdioHandler callback
    void handleTerminalInput(const std::span<const uint8_t> &input) override;
    void handleTerminalResize(uint16_t newRows, uint16_t newCols) override;
    bool checkStdinReady() override { return client_.stdinCanWrite(); }
    void handleStdinNotifyRequest() override { client_.stdinNotifyWhenReady(); }

    // Implement callbacks for RemoteTTYClient
    void handleStdout(const std::span<const uint8_t> &data) override { stdioManager_.writeStdout(data); }
    void handleStderr(const std::span<const uint8_t> &data) override { stdioManager_.writeStderr(data); }
    void handleStdinReady() override { stdioManager_.notifyStdinReady(); };

private:
    std::shared_ptr<Canmore::LinuxClient> linuxRegClient_;
    Canmore::RemoteTTYClient client_;
    RemoteTTYStdioManager stdioManager_;
    bool stopReceived_ = false;
    uint16_t windowRows_ = 0;
    uint16_t windowCols_ = 0;
    std::string termName_;
    std::string cmd_;
    bool useUploadWorkingDir_ = false;
};
