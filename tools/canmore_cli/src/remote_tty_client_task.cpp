#include "RemoteTTYClientTask.hpp"
#include "TerminalDraw.hpp"

#include "canmore/remote_tty_interface.h"

#include <iostream>
#include <signal.h>
#include <sstream>
#include <stdlib.h>
#include <string.h>
#include <sys/eventfd.h>
#include <sys/ioctl.h>
#include <termios.h>
#include <unistd.h>

// ========================================
// RemoteTTYStdioManager Implementation
// ========================================

RemoteTTYStdioManager *RemoteTTYStdioManager::winchReceiver = nullptr;
void RemoteTTYStdioManager::winchHandlerStatic(int sig) {
    if (sig != SIGWINCH)
        return;
    if (winchReceiver != nullptr) {
        winchReceiver->winchHandler();
    }
}

RemoteTTYStdioManager::RemoteTTYStdioManager(RemoteTTYStdioHandler &handler, unsigned int maxInputSize):
    handler_(handler), maxInputSize_(maxInputSize) {
    winchEventFd_ = eventfd(0, EFD_NONBLOCK);
    if (winchEventFd_ < 0) {
        throw std::system_error(errno, std::generic_category(), "eventfd");
    }

    stdinPollDescriptor_ = Canmore::PollFDDescriptor::create(*this, STDIN_FILENO, POLLIN);
    winchEventPollDescriptor_ = Canmore::PollFDDescriptor::create(*this, winchEventFd_, POLLIN);
}

RemoteTTYStdioManager::~RemoteTTYStdioManager() {
    cleanupTerm();
    if (winchEventFd_ >= 0) {
        close(winchEventFd_);
    }
}

void RemoteTTYStdioManager::initTerm() {
    // Don't initialize if we've already been initialized (or else we'll overwrite oldt)
    if (stdioConfigured_)
        return;

    // Backup old flags
    if (tcgetattr(STDIN_FILENO, &oldt_) < 0) {
        throw std::system_error(errno, std::generic_category(), "tcgetattr");
    }

    // Set terminal to raw mode (we want all I/O to be directly passed to the remote client)
    struct termios newt = oldt_;
    cfmakeraw(&newt);
    if (tcsetattr(STDIN_FILENO, TCSANOW, &newt) < 0) {
        throw std::system_error(errno, std::generic_category(), "tcsetattr");
    }

    // Clear the window change eventFd counter for window change events if there's any residual data in it
    // We set it to nonblocking, so it's fine to just do this. Ignore errors since this will give EAGAIN if its empty
    uint64_t garbageData;
    read(winchEventFd_, &garbageData, sizeof(garbageData));

    // Set up the window size change signal handler
    oldWinchReceiver_ = winchReceiver;
    winchReceiver = this;
    struct sigaction newAction = {};
    newAction.sa_handler = &RemoteTTYStdioManager::winchHandlerStatic;
    newAction.sa_flags = 0;
    if (sigaction(SIGWINCH, &newAction, &oldWinchAction_)) {
        winchReceiver = oldWinchReceiver_;
        throw std::system_error(errno, std::generic_category(), "signal");
    }

    // Reset the color on stdout
    std::cout << "\033[0m" << std::flush;

    stdioConfigured_ = true;
}

void RemoteTTYStdioManager::cleanupTerm() noexcept {
    // Don't clean up if we aren't configured right now
    if (!stdioConfigured_)
        return;

    // Reset the color on stdout
    std::cout << "\033[0m" << std::flush;

    // Restore the old terminal behavior
    tcsetattr(STDIN_FILENO, TCSANOW, &oldt_);
    sigaction(SIGWINCH, &oldWinchAction_, NULL);
    winchReceiver = oldWinchReceiver_;

    stdioConfigured_ = false;
}

void RemoteTTYStdioManager::writeStdout(const std::span<const uint8_t> &output) {
    int rc = write(STDOUT_FILENO, output.data(), output.size());
    if (rc < 0 || (unsigned int) rc != output.size()) {
        throw std::system_error(errno, std::generic_category(), "stdout write");
    }
}

void RemoteTTYStdioManager::writeStderr(const std::span<const uint8_t> &output) {
    int rc = write(STDERR_FILENO, output.data(), output.size());
    if (rc < 0 || (unsigned int) rc != output.size()) {
        throw std::system_error(errno, std::generic_category(), "stderr write");
    }
}

void RemoteTTYStdioManager::notifyStdinReady() {
    stdinPollDescriptor_->setEnabled(true);
}

void RemoteTTYStdioManager::populateFds(std::vector<std::weak_ptr<Canmore::PollFDDescriptor>> &descriptors) {
    descriptors.push_back(stdinPollDescriptor_);
    descriptors.push_back(winchEventPollDescriptor_);
}

void RemoteTTYStdioManager::winchHandler() {
    // We need to use an eventfd since the signal can interrupt whenever
    // Just post to the eventfd and let the poll event loop actually handle the event
    uint64_t data = 1;
    if (write(winchEventFd_, &data, sizeof(data)) != sizeof(data)) {
        int writeErrno = errno;
        const char *errmsg = "Exception in Signal Handler! Failed to write winch event fd: ";
        write(STDERR_FILENO, errmsg, strlen(errmsg));
        const char *errdesc = strerror(writeErrno);
        write(STDERR_FILENO, errdesc, strlen(errdesc));
        char newline = '\n';
        write(STDERR_FILENO, &newline, 1);
        abort();
    }
}

void RemoteTTYStdioManager::handleEvent(const pollfd &fd) {
    if (fd.fd == STDIN_FILENO && (fd.revents & POLLIN) != 0) {
        if (handler_.checkStdinReady()) {
            // Allocate buffer for stdin
            std::vector<uint8_t> buf(maxInputSize_);

            // Read up to our internal buffer size
            int rc = read(STDIN_FILENO, buf.data(), buf.size());
            if (rc <= 0) {
                throw std::system_error(errno, std::generic_category(), "stdin read");
            }

            // Resize the buffer to the actual number of bytes read, then call abstract terminal input handler
            buf.resize(rc);
            handler_.handleTerminalInput(buf);
        }
        else {
            // Terminal not ready, request that this be woken up once the output buffer is ready
            stdinPollDescriptor_->setEnabled(false);
            handler_.handleStdinNotifyRequest();
        }
    }
    else if (fd.fd == winchEventFd_ && (fd.revents & POLLIN) != 0) {
        uint64_t data;
        if (read(winchEventFd_, &data, sizeof(data)) != sizeof(data)) {
            throw std::system_error(errno, std::generic_category(), "winch eventfd read");
        }

        struct winsize w;
        if (ioctl(STDOUT_FILENO, TIOCGWINSZ, &w)) {
            throw std::system_error(errno, std::generic_category(), "ioctl TIOCGWINSZ");
        }

        handler_.handleTerminalResize(w.ws_row, w.ws_col);
    }
    else {
        throw std::runtime_error("Unexpected poll event " + std::to_string(fd.revents) + " from fd " +
                                 std::to_string(fd.fd));
    }
}

// ========================================
// RemoteTTYClient Implementation
// ========================================

RemoteTTYClientTask::RemoteTTYClientTask(std::shared_ptr<Canmore::LinuxClient> linuxRegClient,
                                         std::shared_ptr<Canmore::RegMappedCANClient> canClient, const std::string &cmd,
                                         bool useUploadWorkingDir):
    linuxRegClient_(linuxRegClient),
    client_(*this, canClient->ifIndex, canClient->clientId), stdioManager_(*this, client_.getMaxFrameSize()), cmd_(cmd),
    useUploadWorkingDir_(useUploadWorkingDir) {}

void RemoteTTYClientTask::run() {
    if (linuxRegClient_->remoteTtyEnabled()) {
        std::cout << COLOR_NOTICE << "Resuming existing remote tty connection..." COLOR_RESET << std::endl;
    }
    else {
        // Try to pull terminal data. If it fails, fall back to null defaults set in header definition
        const char *termEnv = getenv("TERM");
        if (termEnv) {
            termName_ = termEnv;
        }
        struct winsize w;
        if (ioctl(STDOUT_FILENO, TIOCGWINSZ, &w) == 0) {
            windowRows_ = w.ws_row;
            windowCols_ = w.ws_col;
        }

        // Startup remote tty on server side
        linuxRegClient_->enableRemoteTty(termName_, windowRows_, windowCols_, cmd_, useUploadWorkingDir_);
    }

    // Initialize terminal and event loops
    stdioManager_.initTerm();
    Canmore::PollGroup grp;
    grp.addFd(stdioManager_);
    grp.addFd(client_);

    // Loop until we disconnect
    bool disconnectedInError = false;
    while (!stopReceived_ && !client_.isDisconnected(disconnectedInError)) {
        grp.processEvent(2000);
    }

    // Send disconnect if we got a stop received
    if (!client_.isDisconnected(disconnectedInError)) {
        client_.disconnect(false);
    }

    stdioManager_.cleanupTerm();

    // Giving a lot of unnecessary spam, just don't show it until this is figured out
    // if (disconnectedInError) {
    //     std::cout << COLOR_NOTICE "Host disconnected due to error!" COLOR_RESET << std::endl;
    // }
}

void RemoteTTYClientTask::handleTerminalInput(const std::span<const uint8_t> &input) {
    client_.stdinWrite(input);
}

void RemoteTTYClientTask::handleTerminalResize(uint16_t newRows, uint16_t newCols) {
    if (newRows != windowRows_ || newCols != windowCols_) {
        client_.sendWindowSize(newRows, newCols);
        windowRows_ = newRows;
        windowCols_ = newCols;
    }
}
