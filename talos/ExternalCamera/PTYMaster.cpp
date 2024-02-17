#include "DFCDaemon.hpp"

CanmoreTTYServer::CanmoreTTYServer(int ifIndex, uint8_t clientId, const std::string &termEnv, uint16_t initialRows,
                                   uint16_t initialCols):
    canmoreServer_(*this, ifIndex, clientId) {}

void CanmoreTTYServer::handleStdin(const std::span<const uint8_t> &data) {
    canmoreServer_.stdoutWrite(data);

    if (std::find(data.begin(), data.end(), 2) != data.end()) {
        canmoreServer_.disconnect(true);
    }
    if (std::find(data.begin(), data.end(), 1) != data.end()) {
        canmoreServer_.disconnect(false);
    }
}

void CanmoreTTYServer::populateFds(std::vector<std::weak_ptr<Canmore::PollFDDescriptor>> &descriptors) {
    canmoreServer_.populateFds(descriptors);
}

void CanmoreTTYServer::handleWindowSize(uint16_t rows, uint16_t cols) {
    unsigned char data[] = "WDWSZ\r\n";
    canmoreServer_.stderrWrite({ data });
}

void CanmoreTTYServer::handleStdioReady() {
    // TODO: Implement me
}
