#include "CanmoreCLI.hpp"
#include "TerminalDraw.hpp"

#include <chrono>
#include <cxxabi.h>
#include <iostream>
#include <net/if.h>
#include <thread>

using namespace Canmore;

bool isValidCANDevice(const char *name) {
    if (name[0] == 'v')
        name++;
    return (name[0] == 'c' && name[1] == 'a' && name[2] == 'n' && (name[3] >= '0' && name[3] <= '9'));
}

int main(int argc, char **argv) {
    if (argc == 2) {
        renderHeader("Usage");
        std::cout << COLOR_NAME << argv[0] << COLOR_RESET << std::endl;
        std::cout << "\tRun CANmore CLI in interative mode" << std::endl;
        std::cout << std::endl;
        std::cout << COLOR_NAME << argv[0] << COLOR_BODY " [Target Device] [Command ...]" COLOR_RESET << std::endl;
        std::cout << "\tRun the requested CLI command on the specified target" << std::endl;
        std::cout << "\tSetting the environment variable " COLOR_BODY "CANMORECLI_QUIET=1" COLOR_RESET
                     " will silence connect messages"
                  << std::endl;
        std::cout << std::endl;
        return 1;
    }

    try {
        auto devMap = DeviceMap::create();

        // ===== Discover devices =====
        std::vector<std::shared_ptr<Discovery>> discoverySources;
        discoverySources.push_back(EthernetDiscovery::create());

        // Discover all CAN interfaces
        struct if_nameindex *if_nidxs, *intf;
        if_nidxs = if_nameindex();
        if (if_nidxs != NULL) {
            for (intf = if_nidxs; intf->if_index != 0 || intf->if_name != NULL; intf++) {
                if (isValidCANDevice(intf->if_name)) {
                    discoverySources.push_back(CANDiscovery::create(intf->if_index));
                }
            }
            if_freenameindex(if_nidxs);
        }

        if (argc < 3) {
            // No arguments provided, run in interactive mode
            // Get user device selection
            while (true) {
                auto device = getTargetDevice(devMap, discoverySources);
                if (!device)
                    break;

                runCli(device);
            }
        }
        else {
            std::string targetName = argv[1];
            uint64_t targetSerialNum = devMap.lookupSerialByName(targetName);
            // Client requested, run the specified command
            auto device = waitForTargetDevice(discoverySources, targetSerialNum);
            if (!device) {
                std::cerr << "Failed to find target " << targetName << std::endl;
                return 1;
            }

            std::vector<std::string> args;
            if (argc > 3) {
                args.reserve(argc - 3);
            }
            for (int i = 3; i < argc; i++) {
                args.push_back(argv[i]);
            }

            runCliCommand(device, argv[2], args);
        }
    } catch (std::exception &e) {
        int status;
        char *exceptionName = abi::__cxa_demangle(abi::__cxa_current_exception_type()->name(), 0, 0, &status);
        std::cerr << std::endl
                  << "[EXCEPTION] Exception '" << exceptionName << "' caused program termination" << std::endl;
        free(exceptionName);
        std::cerr << "  what():  " << e.what() << std::endl;
    } catch (...) {
        std::cerr << std::endl << "[EXCEPTION] Unknown exception caused program termination" << std::endl;
    }

    return 0;
}
