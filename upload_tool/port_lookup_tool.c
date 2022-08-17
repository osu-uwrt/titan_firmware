#include <stdio.h>

#include "uploader.h"

bool verbose = false;

int main(int argc, char** argv) {
    if (argc < 3) {
        fprintf(stderr, "Invalid syntax! Usage: %s [board type] [interface name]\n", argv[0]);
        return 1;
    }

    int ret = -1;
    libusb_context* ctx;
    if (libusb_init(&ctx) != LIBUSB_SUCCESS) {
        fprintf(stderr, "Failed to init libusb!");
        return 1;
    }

    rp2040_discovery_ctx_t* device_discovery_ctx = NULL;
    rp2040_device_t *device = NULL;
    if (device_discovery_ctx != NULL){
        cleanup_rp2040_discovery(device_discovery_ctx);
    }
    device_discovery_ctx = discover_rp2040(ctx);
    device = get_rp2040_device(device_discovery_ctx, argv[1], true);

    if (device == NULL){
        fprintf(stderr, "Failed to find RP2040 device\n");
        cleanup_rp2040_discovery(device_discovery_ctx);
        goto cleanup_ctx;
    }

    char portname[50];
    if (get_serial_port(device, argv[2], portname, sizeof(portname))) {
        printf("%s\n", portname);
        ret = 0;
    } else {
        fprintf(stderr, "CDC Serial interface '%s' not found\n", argv[2]);
        ret = 1;
    }

    cleanup_rp2040_discovery(device_discovery_ctx);

cleanup_ctx:
    libusb_exit(ctx);
    return ret;
}
