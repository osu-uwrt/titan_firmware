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

static const char* const positionalArgsNames[] = {"uf2"};
static const size_t numPositionalArgs = sizeof(positionalArgsNames)/sizeof(*positionalArgsNames);

class CANBlToolArgs {
public:
    // Arguments
    bool waitInBootDelay;
    bool justPullInfo;
    bool allowBootloaderOverwrite;
    bool showHelpAndQuit;
    const char *filename;
    const char *progname;

    bool parseSuccessful;

    CANBlToolArgs(int argc, char** argv):
            // Define default args
            waitInBootDelay(false), justPullInfo(false), allowBootloaderOverwrite(false), filename(""),

            // Attributes
            progname("[???]"), parseSuccessful(false), argc(argc), argv(argv), positionalIndex(0) {

        // Do parse
        parseSuccessful = tryParse();
    }

    void printHelp() {
        std::cout << "Usage: " << progname << " [-fiw] [uf2]" << std::endl;
        std::cout << "\t-h: Show this help message" << std::endl;
        std::cout << "\t-f: Full Image Flash (if omitted, uf2 is assumed ota file)" << std::endl;
        std::cout << "\t\tAllows flashing of images containing a bootloader rather than restricting to OTA" << std::endl;
        std::cout << "\t-i: Print Info" << std::endl;
        std::cout << "\t\tPrints information from the passed file and quits" << std::endl;
        std::cout << "\t-w: Wait for Boot" << std::endl;
        std::cout << "\t\tPrompts to wait for device in boot" << std::endl;
        std::cout << "\tuf2: A UF2 file to flash" << std::endl;
    }

private:
    int argc;
    char** argv;
    size_t positionalIndex;

    const char* tryGetArgument(const char *argname) {
        if (!argc) {
            std::cout << "Error: Expected argument '" << argname << "'" << std::endl;
            return NULL;
        }
        argc--;
        return *argv++;
    }

    bool tryParsePositional(const char *arg) {
        switch (positionalIndex) {
            case 0:
                filename = arg;
                break;
            default:
                std::cout << "Unexpected positional argument '" << arg << "'" << std::endl;
                return false;
        }
        positionalIndex++;
        return true;
    }


    bool tryParse() {
        const char* prognameTemp = tryGetArgument("progname"); \
        if (!prognameTemp) {return false;}
        progname = prognameTemp;

        // Loop until all positional arguments found (or help, which is special)
        while ((positionalIndex < numPositionalArgs || argc > 0) && !showHelpAndQuit) {
            const char* arg = tryGetArgument(positionalArgsNames[positionalIndex]);
            if (!arg) return false;

            // Check if flag
            if (arg[0] == '-') {
                if (arg[2] != '\0') {
                    std::cout << "Flag '" << arg << "' is not single character" << std::endl;
                    return false;
                }

                switch (arg[1]) {
                case 'f':
                    allowBootloaderOverwrite = true;
                    break;
                case 'i':
                    justPullInfo = true;
                    break;
                case 'w':
                    waitInBootDelay = true;
                    break;
                case 'h':
                    showHelpAndQuit = true;
                    break;
                default:
                    std::cout << "Unexpected flag: -" << arg[1] << std::endl;
                    return false;
                }
            }
            // If not it's a positional argument
            else {
                if (!tryParsePositional(arg)) {
                    return false;
                }
            }
        }
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
        std::cout << "Run '" << blArgs.progname << " -h' to show help." << std::endl;
        return 1;
    }
    if (blArgs.showHelpAndQuit) {
        blArgs.printHelp();
        return 0;
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
        interface = UploadTool::catchInBootDelay(discoverySources, devMap, uf2);
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