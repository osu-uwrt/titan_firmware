#include "DFCDaemon.hpp"

#include <iostream>
#include <net/if.h>

int main(int argc, char **argv) {
    if (argc < 2) {
        std::cerr << "Expected interface name argument" << std::endl;
        return 1;
    }

    int ifIdx = if_nametoindex(argv[1]);
    if (!ifIdx) {
        throw std::system_error(errno, std::generic_category(), "if_nametoindex");
    }

    HeartbeatTransmitter heartbeatTx(ifIdx, 6);
    heartbeatTx.start();

    CanmoreLinuxServer server(ifIdx, 6);
    CanmoreTTYServer ttyServer(ifIdx, 6);

    Canmore::PollGroup group;
    group.addFd(server);
    group.addFd(ttyServer);

    while (!server.stopRequested()) {
        group.processEvent(1000);
    }

    return 0;
}
