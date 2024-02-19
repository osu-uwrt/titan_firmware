#include "DFCDaemon.hpp"

#include <iostream>
#include <net/if.h>
#include <signal.h>

// TODO: Define client id
#define clientId 6

// ========================================
// Shutdown Handling
// ========================================

static CanmoreLinuxServer *signalServerPtr = nullptr;
static void shutdownHandler(int signum) {
    // Send a tty shutdown so we don't lock up the client thinking it just lost comms

    if (signalServerPtr) {
        signalServerPtr->forceTTYdisconnect();
        signalServerPtr = nullptr;  // Deregister so if we manage to get multiple fires, it won't go through
    }

    // Throw the signal again (and it's oneshot so it won't fire again)
    kill(getpid(), signum);
}

static void installShutdownHandler(CanmoreLinuxServer *server) {
    signalServerPtr = server;

    struct sigaction sig = {};
    sig.sa_handler = &shutdownHandler;
    sig.sa_flags = SA_RESETHAND;  // Need to restore so when we rethrow the signal it'll actually go through
    sigfillset(&sig.sa_mask);     // Don't allow preemption by ANY signal, since we're shutting it all down

    // Catch all the signals where we will die, but trust the program state enough that we can send a disconnect packet
    sigaction(SIGTERM, &sig, NULL);
    sigaction(SIGHUP, &sig, NULL);
    sigaction(SIGINT, &sig, NULL);
    sigaction(SIGQUIT, &sig, NULL);
}

static void cleanupShutdownHandler() {
    signalServerPtr = nullptr;
}

// ========================================
// Main Function
// ========================================

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

    // Start the heartbeat and kick any canmore cli's out of tty mode (as we aren't in one right now)
    heartbeatTx.start();
    regMappedServer.forceTTYdisconnect();
    installShutdownHandler(&regMappedServer);

    try {
        while (!regMappedServer.stopRequested()) {
            group.processEvent(1000);

            // Handle starting up/shutting down the tty server
            if (regMappedServer.getTtyEnabled()) {
                if (!ttyServer) {
                    // TTY enabled, but no server. That means we need to start it
                    // Grab the initial config from the reg mapped server
                    std::string termName;
                    uint16_t initialRows;
                    uint16_t initialCols;
                    regMappedServer.getTtyInitialConfig(termName, initialRows, initialCols);

                    ttyServer = std::make_shared<CanmoreTTYServer>(ifIdx, clientId, termName, initialRows, initialCols);
                    group.addFd(*ttyServer);
                    std::cout << "TTY Server Started" << std::endl;
                }
                else {
                    bool termInError;
                    if (ttyServer->isTerminated(termInError)) {
                        // TTY was enabled, but the server shut down
                        // Notify the reg mapped server that we shut down, then destroy the tty server
                        std::cout << "TTY Server terminated " << (termInError ? "in error state" : "cleanly")
                                  << std::endl;
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
    } catch (...) {
        // If we die, send a disconnect to make the client aware that we're going away
        cleanupShutdownHandler();
        regMappedServer.forceTTYdisconnect();
        throw;
    }

    cleanupShutdownHandler();
    return 0;
}
