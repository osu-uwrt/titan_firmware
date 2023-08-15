#include "CanmoreCLI.hpp"

#include <chrono>
#include <cxxabi.h>
#include <iostream>
#include <net/if.h>
#include <thread>

using namespace Canmore;

bool isValidCANDevice(const char *name) {
    return (name[0] == 'c' && name[1] == 'a' && name[2] == 'n' && (name[3] >= '0' && name[3] <= '9'));
}

int main() {
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

        // Get user device selection
        while (true) {
            auto device = getTargetDevice(devMap, discoverySources);
            if (!device)
                break;

            runCli(device);
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
