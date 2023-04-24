#include <iostream>
#include <filesystem>
#include <memory>
#include <vector>
#include <thread>
#include <net/if.h>

#include "RP2040FlashInterface.hpp"
#include "canmore_titan/protocol.h"
#include "canmore_cpp/Discovery.hpp"
#include "canmore_cpp/BootloaderClient.hpp"
#include "canmore_cpp/RegMappedClient.hpp"

#ifndef DEVICE_DEFINITIONS_FILE
#error DEVICE_DEFINITIONS_FILE must be defined as json file relative to application
#endif

char version_str[1024];

namespace fs = std::filesystem;

void DumpHex(const void* data, size_t size) {
	char ascii[17];
	size_t i, j;
	ascii[16] = '\0';
	for (i = 0; i < size; ++i) {
		printf("%02X ", ((unsigned char*)data)[i]);
		if (((unsigned char*)data)[i] >= ' ' && ((unsigned char*)data)[i] <= '~') {
			ascii[i % 16] = ((unsigned char*)data)[i];
		} else {
			ascii[i % 16] = '.';
		}
		if ((i+1) % 8 == 0 || i+1 == size) {
			printf(" ");
			if ((i+1) % 16 == 0) {
				printf("|  %s \n", ascii);
			} else if (i+1 == size) {
				ascii[(i+1) % 16] = '\0';
				if ((i+1) % 16 <= 8) {
					printf(" ");
				}
				for (j = (i+1) % 16; j < 16; ++j) {
					printf("   ");
				}
				printf("|  %s \n", ascii);
			}
		}
	}
}

bool isValidCANDevice(const char *name) {
    return (name[0] == 'c' && name[1] == 'a' && name[2] == 'n' &&
            (name[3] >= '0' && name[3] <= '9'));
}

class CANBlToolArgs {
public:
    // Arguments
    bool waitInBootDelay;
    bool justPullInfo;
    bool allowBootloaderOverwrite;
    const char *filename;

    bool parseSuccessful;

    CANBlToolArgs(int argc, char** argv):
            // Define default args
            waitInBootDelay(false), justPullInfo(false), allowBootloaderOverwrite(false), filename(""),

            // Private Attributes
            parseSuccessful(false), progname("[???]"), argc(argc), argv(argv) {

        // Do parse
        parseSuccessful = tryParse();

        if (!parseSuccessful) {
            std::cout << std::endl;
            printHelp();
        }
    }

private:
    const char *progname;
    int argc;
    char** argv;

    void printHelp() {
        std::cout << "Usage: " << progname << " [-fiw] [filename]" << std::endl;
        std::cout << "\t-f: Full Image Flash" << std::endl;
        std::cout << "\t\tAllows flashing of images containing a bootloader rather than restring to OTA" << std::endl;
        std::cout << "\t-i: Print Info" << std::endl;
        std::cout << "\t\tPrints information from the passed file and quits" << std::endl;
        std::cout << "\t-w: Wait for Boot" << std::endl;
        std::cout << "\t\tPrompts to wait for device in boot" << std::endl;
        std::cout << "\tfilename: A UF2 file to flash" << std::endl;
    }

    const char* tryGetArgument(const char *argname) {
        if (!argc) {
            std::cout << "Error: Expected argument '" << argname << "'" << std::endl;
            return NULL;
        }
        argc--;
        return *argv++;
    }

    #define getArgumentOrReturn(variable) \
        variable = tryGetArgument(#variable); \
        if (!variable) {return false;}

    #define defArgumentOrReturn(variable) \
        const char* getArgumentOrReturn(variable)

    bool tryParse() {
        getArgumentOrReturn(progname);
        while (argc > 1) {
            defArgumentOrReturn(flag);
            // Sanity check that its actually a flag
            if (flag[0] != '-' || flag[2] != '\0') {
                std::cout << "Expected flag, not '" << flag << "'" << std::endl;
                return false;
            }

            switch (flag[1]) {
            case 'f':
                allowBootloaderOverwrite = true;
                break;
            case 'i':
                justPullInfo = true;
                break;
            case 'w':
                waitInBootDelay = true;
                break;
            default:
                std::cout << "Unexpected flag: -" << flag[1] << std::endl;
                return false;
            }
        }

        getArgumentOrReturn(filename);
        return true;
    }
};

int main(int argc, char** argv) {
    // Load in device map
    fs::path applicationPath = argv[0];
    auto deviceDefinitionPath = applicationPath.parent_path().append(DEVICE_DEFINITIONS_FILE);
    if (!fs::exists(deviceDefinitionPath)) {
        throw std::runtime_error("Cannot locate device file '" DEVICE_DEFINITIONS_FILE "' in application directory");
    }
    UploadTool::DeviceMap devMap(deviceDefinitionPath);

    // Pull arguments
    CANBlToolArgs blArgs(argc, argv);
    if (!blArgs.parseSuccessful) {
        return 1;
    }

    // Load in file
    UploadTool::RP2040UF2 uf2(blArgs.filename);

    if (blArgs.justPullInfo) {
        UploadTool::dumpUF2(uf2);
        return 0;
    }

    // ===== Discover devices =====
    std::vector<std::shared_ptr<UploadTool::RP2040Discovery>> discoverySources;
    discoverySources.push_back(Canmore::EthernetDiscovery::create());

    // Discover all CAN interfaces
    struct if_nameindex *if_nidxs, *intf;
    if_nidxs = if_nameindex();
    if (if_nidxs != NULL)
    {
        for (intf = if_nidxs; intf->if_index != 0 || intf->if_name != NULL; intf++)
        {
            if (isValidCANDevice(intf->if_name)) {
                discoverySources.push_back(Canmore::CANDiscovery::create(intf->if_index));
            }
        }
        if_freenameindex(if_nidxs);
    }

    std::shared_ptr<UploadTool::RP2040FlashInterface> interface;
    if (blArgs.waitInBootDelay) {
        interface = UploadTool::catchInBootDelay(discoverySources, devMap, uf2.boardType);
    }
    else {
        // Wait for devices to appear
        std::cout << "Waiting for devices..." << std::endl;
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));

        // Find all discovered devices
        std::vector<std::shared_ptr<UploadTool::RP2040Device>> discovered;
        for (auto discovery : discoverySources) {
            std::vector<std::shared_ptr<UploadTool::RP2040Device>> interfaceDiscovered;
            discovery->discoverDevices(interfaceDiscovered);
            discovered.insert(std::end(discovered), std::begin(interfaceDiscovered), std::end(interfaceDiscovered));
        }

        if (discovered.size() == 0) {
            std::cout << "No devices to select" << std::endl;
            return 0;
        }

        auto dev = UploadTool::selectDevice(discovered, devMap, uf2.boardType, true);
        interface = dev->getFlashInterface();
    }

    // std::array<uint8_t, UF2_PAGE_SIZE> my_page;
    // interface->readBytes(0x10000000, my_page);
    // DumpHex(my_page.data(), my_page.size());

    UploadTool::flashImage(interface, uf2, !blArgs.allowBootloaderOverwrite);

    return 0;
}