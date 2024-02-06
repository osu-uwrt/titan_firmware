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
    server.run();

    return 0;
}
