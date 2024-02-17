#include "DFCDaemon.hpp"

#include <iostream>
#include <net/if.h>

#define clientId 6

int main(int argc, char **argv) {
    if (argc < 2) {
        std::cerr << "Expected interface name argument" << std::endl;
        return 1;
    }

    int ifIdx = if_nametoindex(argv[1]);
    if (!ifIdx) {
        throw std::system_error(errno, std::generic_category(), "if_nametoindex");
    }

    HeartbeatTransmitter heartbeatTx(ifIdx, clientId);
    CanmoreLinuxServer regMappedServer(ifIdx, clientId);
    std::shared_ptr<CanmoreTTYServer> ttyServer;

    Canmore::PollGroup group;
    group.addFd(regMappedServer);

    heartbeatTx.start();

    while (!regMappedServer.stopRequested()) {
        group.processEvent(1000);

        // Handle starting up/shutting down the tty server
        if (regMappedServer.getTtyEnabled()) {
            if (!ttyServer) {
                // TTY enabled, but no server. That means we need to start it
                // Grab the initial config from the reg mapped server
                std::string startName;
                uint16_t initialRows;
                uint16_t initialCols;
                regMappedServer.getTtyInitialConfig(startName, initialRows, initialCols);

                ttyServer = std::make_shared<CanmoreTTYServer>(ifIdx, clientId);
                group.addFd(*ttyServer);
                std::cout << "TTY Server Started" << std::endl;
            }
            else {
                bool termInError;
                if (ttyServer->isTerminated(termInError)) {
                    // TTY was enabled, but the server shut down
                    // Notify the reg mapped server that we shut down, then destroy the tty server
                    std::cout << "TTY Server terminated " << (termInError ? "in error state" : "cleanly") << std::endl;
                    regMappedServer.notifyTtyShutdown();
                    ttyServer.reset();
                }
            }
        }
        else {
            if (ttyServer) {
                // Reg mapped server must have shut off the tty, destroy the tty server
                std::cout << "TTY Server terminated via reg mapped request" << std::endl;
                ttyServer.reset();
            }
        }
    }

    return 0;
}
