#include <stdio.h>
#include <net/if.h>

#include "canmore_titan/protocol.h"
#include "reg_mapped_client_linux.h"
#include "bl_client.h"

char version_str[1024];

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

    unsigned int if_index = if_nametoindex(argv[1]);

    if (!if_index) {
        perror("if_nametoindex");
        return 1;
    }

    reg_mapped_client_linux_inst_t comm_inst;
    if (!reg_mapped_client_linux_open(&comm_inst, if_index, 4, CANMORE_TITAN_CHAN_CONTROL_INTERFACE, 50)) {
        printf("Failed to initialize linux client!\n");
        return 1;
    }

    bl_client_inst_t bl_client_inst;
    if (!bl_client_init(&bl_client_inst, reg_mapped_client_linux_get_client(&comm_inst))) {
        reg_mapped_client_linux_close(&comm_inst);
        printf("Failed to initialize bootloader client\n");
        return 1;
    }

    union {
        uint64_t doubleword;
        uint8_t bytes[8];
    } flash_id;

    flash_id.doubleword = bl_client_get_flash_id(&bl_client_inst);

    size_t i;
    // Generate hex one nibble at a time
    char id_out[17];
    for (i = 0; (i < sizeof(flash_id) * 2); i++) {
        int nibble = (flash_id.bytes[i/2] >> (4 - 4 * (i&1))) & 0xf;
        id_out[i] = (char)(nibble < 10 ? nibble + '0' : nibble + 'A' - 10);
    }
    id_out[i] = 0;

    int result;
    if ((result = bl_client_get_version(&bl_client_inst, version_str, sizeof(version_str))) < 0) {
        printf("Failed to read version string!: %d\n", result);
        reg_mapped_client_linux_close(&comm_inst);
        return 1;
    }

    printf("Board ID: %s\n", id_out);
    printf("Version String: %s\n", version_str);

    reg_mapped_client_linux_close(&comm_inst);
    return 0;
}