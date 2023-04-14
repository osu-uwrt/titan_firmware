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

int main(int argc, char** argv) {
    // struct if_nameindex *if_nidxs, *intf;

    // if_nidxs = if_nameindex();
    // if ( if_nidxs != NULL )
    // {
    //     for (intf = if_nidxs; intf->if_index != 0 || intf->if_name != NULL; intf++)
    //     {
    //         printf("%s\n", intf->if_name);
    //     }

    //     if_freenameindex(if_nidxs);
    // }

    // Load in device map
    fs::path applicationPath = argv[0];
    auto deviceDefinitionPath = applicationPath.parent_path().append(DEVICE_DEFINITIONS_FILE);
    if (!fs::exists(deviceDefinitionPath)) {
        throw std::runtime_error("Cannot locate device file '" DEVICE_DEFINITIONS_FILE "' in application directory");
    }
    UploadTool::DeviceMap devMap(deviceDefinitionPath);

    if (argc != 3) {
        printf("Usage: %s [interface] [filename]\n", argv[0]);
        return 1;
    }

    UploadTool::RP2040UF2 uf2(argv[2]);

    // unsigned int ifIndex = if_nametoindex(argv[1]);
    // if (!ifIndex) {
    //     perror("if_nametoindex");
    //     return 1;
    // }

    // auto discovery = Canmore::CANDiscovery::create(ifIndex);

    auto discovery = Canmore::EthernetDiscovery::create();

    std::cout << "Waiting for devices..." << std::endl;
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));

    std::vector<std::shared_ptr<UploadTool::RP2040Device>> discovered;
    discovery->discoverDevices(discovered);
    if (discovered.size() == 0) {
        std::cout << "No devices to select" << std::endl;
        auto interface = discovery->catchDeviceInBootDelay(42);

        std::cout << "Version: " << interface->getVersion() << std::endl;
        return 0;
    }

    auto dev = UploadTool::selectDevice(discovered, devMap, uf2.boardType, true);
    auto interface = dev->getFlashInterface();

    // std::array<uint8_t, UF2_PAGE_SIZE> my_page;
    // interface->readBytes(0x10000000, my_page);
    // DumpHex(my_page.data(), my_page.size());

    UploadTool::flashImage(interface, uf2, true);

    interface->reboot();

    return 0;
}