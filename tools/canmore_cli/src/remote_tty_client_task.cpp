#include "RemoteTTYClientTask.hpp"
#include "TerminalDraw.hpp"

#include "titan/canmore.h"

#include <iostream>
#include <sstream>
#include <termios.h>
#include <unistd.h>

// ========================================
// RemoteTTYStdioManager Implementation
// ========================================

void RemoteTTYStdioManager::initTerm() {
    // Don't initialize if we've already been initialized (or else we'll overwrite oldt)
    if (stdioConfigured)
        return;

    // Backup old flags
    if (tcgetattr(STDIN_FILENO, &oldt) < 0) {
        throw std::system_error(errno, std::generic_category(), "tcgetattr");
    }

    // Set terminal to raw mode (we want all I/O to be directly passed to the remote client)
    struct termios newt = oldt;
    cfmakeraw(&newt);
    if (tcsetattr(STDIN_FILENO, TCSANOW, &newt) < 0) {
        throw std::system_error(errno, std::generic_category(), "tcsetattr");
    }

    // Reset the color on stdout
    std::cout << "\033[0m" << std::flush;

    stdioConfigured = true;
}

void RemoteTTYStdioManager::cleanupTerm() noexcept {
    // Don't clean up if we aren't configured right now
    if (!stdioConfigured)
        return;

    // Reset the color on stdout
    std::cout << "\033[0m" << std::flush;

    // Restore the old terminal value
    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);

    stdioConfigured = false;
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

void RemoteTTYStdioManager::populateFds(std::vector<std::pair<PollFDHandler *, pollfd>> &fds) {
    fds.push_back({ this, { .fd = STDIN_FILENO, .events = POLLIN, .revents = 0 } });
}

void RemoteTTYStdioManager::handleEvent(const pollfd &fd) {
    if (fd.revents & POLLIN) {
        // Allocate buffer for stdin
        std::vector<uint8_t> buf(maxInputSize);

        // Read up to our internal buffer size
        int rc = read(STDIN_FILENO, buf.data(), buf.size());
        if (rc <= 0) {
            throw std::system_error(errno, std::generic_category(), "stdin read");
        }

        // Resize the buffer to the actual number of bytes read, then call abstract terminal input handler
        buf.resize(rc);
        handler.handleTerminalInput(buf);
    }
    else {
        throw std::runtime_error("Unexpected poll event: " + std::to_string(fd.revents));
    }
}

// ========================================
// RemoteTTYClient Implementation
// ========================================

RemoteTTYClientTask::RemoteTTYClientTask(std::shared_ptr<Canmore::RegMappedCANClient> canClient):
    client(*this, canClient->ifIndex, canClient->clientId), stdioManager(*this, CANMORE_FRAME_SIZE) {}

void RemoteTTYClientTask::run() {
    // TODO: Set the TERM environment variable properly

    stdioManager.initTerm();
    Canmore::PollGroup grp;
    grp.addFd(&stdioManager);
    grp.addFd(&client);

    // Loop until we disconnect
    bool disconnectedInError = false;
    while (!stopReceived && !client.isDisconnected(disconnectedInError)) {
        grp.processEvent(2000);
    }

    // Send disconnect if we got a stop received
    if (!client.isDisconnected(disconnectedInError)) {
        client.disconnect(false);
    }

    stdioManager.cleanupTerm();

    if (disconnectedInError) {
        std::cout << COLOR_NOTICE "Host disconnected due to error!" COLOR_RESET << std::endl;
    }
}

void RemoteTTYClientTask::handleTerminalInput(const std::span<const uint8_t> &input) {
    if (client.stdinCanWrite()) {
        client.stdinWrite(input);
    }
    // TODO: Disable reading if the buffer fills up

    if (std::find(input.begin(), input.end(), 3) != input.end()) {
        // We found a Ctrl+C, notify stop
        stopReceived = true;
    }
}

void RemoteTTYClientTask::handleStdinReady() {
    // TODO: Do things
}

void RemoteTTYClientTask::handleStdout(const std::span<const uint8_t> &data) {
    stdioManager.writeStdout(data);
}

void RemoteTTYClientTask::handleStderr(const std::span<const uint8_t> &data) {
    stdioManager.writeStderr(data);
}
