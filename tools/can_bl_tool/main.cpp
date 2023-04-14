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

int main(int argc, char** argv) {
    // Load in device map
    fs::path applicationPath = argv[0];
    auto deviceDefinitionPath = applicationPath.parent_path().append(DEVICE_DEFINITIONS_FILE);
    if (!fs::exists(deviceDefinitionPath)) {
        throw std::runtime_error("Cannot locate device file '" DEVICE_DEFINITIONS_FILE "' in application directory");
    }
    UploadTool::DeviceMap devMap(deviceDefinitionPath);

    // Check arguments
    if ((argc != 2 && argc != 3) || (argc == 3 && argv[1][0] != '-')) {
        printf("Usage: %s (-w) [filename]\n", argv[0]);
        printf("\t-w: Wait for device in boot delay\n");
        return 1;
    }

    // Parse arguments
    bool waitInBootDelay = false;
    const char *filename;
    if (argc == 3) {
        if (!strncmp(argv[1], "-w", 3)) {
            waitInBootDelay = true;
        } else {
            printf("Invalid flag: '%s'\n", argv[1]);
            return 1;
        }
        filename = argv[2];
    } else {
        filename = argv[1];
    }

    // Load in file
    UploadTool::RP2040UF2 uf2(filename);

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
    if (waitInBootDelay) {
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

    UploadTool::flashImage(interface, uf2, true);

    return 0;
}