#include <iostream>
#include <memory>
#include <vector>
#include <net/if.h>

#include "canmore_titan/protocol.h"
#include "RP2040FlashInterface.hpp"
#include "canmore_cpp/BootloaderClient.hpp"
#include "canmore_cpp/RegMappedClient.hpp"

char version_str[1024];

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

    if (argc != 2) {
        printf("Usage: %s [interface]\n", argv[0]);
        return 1;
    }

    unsigned int ifIndex = if_nametoindex(argv[1]);
    if (!ifIndex) {
        perror("if_nametoindex");
        return 1;
    }

    auto blInterface = std::make_shared<Canmore::RegMappedCANClient>(ifIndex, 4, CANMORE_TITAN_CHAN_CONTROL_INTERFACE, 500);
    Canmore::BootloaderClient client(blInterface);

    union {
        uint64_t doubleword;
        uint8_t bytes[8];
    } flash_id;

    flash_id.doubleword = client.getFlashId();
    size_t i;
    // Generate hex one nibble at a time
    char id_out[17];
    for (i = 0; (i < sizeof(flash_id) * 2); i++) {
        int nibble = (flash_id.bytes[i/2] >> (4 - 4 * (i&1))) & 0xf;
        id_out[i] = (char)(nibble < 10 ? nibble + '0' : nibble + 'A' - 10);
    }
    id_out[i] = 0;

    std::string version;
    client.getVersion(version);

    std::cout << "Board ID: " << id_out << std::endl;
    std::cout << "Version: " << version << std::endl;

    std::vector<uint8_t> my_page;
    client.readBytes(0x10000000, my_page);
    DumpHex(my_page.data(), my_page.size());

    client.reboot();
    return 0;
}