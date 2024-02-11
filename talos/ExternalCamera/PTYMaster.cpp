#include "DFCDaemon.hpp"

CanmoreTTYServer::CanmoreTTYServer(int ifIndex, uint8_t clientId): server_(*this, ifIndex, clientId) {}

void CanmoreTTYServer::handleStdin(const std::span<const uint8_t> &data) {
    server_.stdoutWrite(data);
}

void CanmoreTTYServer::populateFds(std::vector<std::pair<Canmore::PollFDHandler *, pollfd>> &fds) {
    server_.populateFds(fds);
}

void CanmoreTTYServer::handleWindowSize(uint16_t rows, uint16_t cols) {
    unsigned char data[] = "WDWSZ\r\n";
    server_.stderrWrite({ data });
}

void CanmoreTTYServer::handleStdioReady() {
    // TODO: Implement me
}
