#include <stdio.h>
#include <stdbool.h>
#include <string.h>
#include <unistd.h>

#include <libusb-1.0/libusb.h>

#include "uploader.h"

#define NUM_RESET_DISCOVERY_TIMEOUT_MS 5000

bool verbose = false;

enum action{ACT_UNSET, ACT_MONITOR, ACT_RESET, ACT_UPLOAD, ACT_RESET_BOOTSEL, ACT_INFO, ACT_HELP};

#define SET_ACTION(value) if (cmd_action == ACT_UNSET || cmd_action == value) { cmd_action = value; } else { goto cmd_multiple; }

int main(int argc, char** argv) {
    libusb_context* ctx;
    int ret = -1;

    // ========================================
    // Command Line Parsing
    // ========================================

    enum action cmd_action = ACT_UNSET;
    bool reset_before_monitor = false;
    bool wait_for_connect = false;
    bool no_interaction = false;
    int wait_timeout = 0;
    const char* firmware_filename = NULL;
    for (int i = 1; i < argc; i++) {
        if (!strcmp("-h", argv[i]) || !strcmp("--help", argv[i])) {
            SET_ACTION(ACT_HELP);
        } else if (!strcmp("-m", argv[i]) || !strcmp("--monitor", argv[i])) {
            if (cmd_action == ACT_RESET) {
                cmd_action = ACT_MONITOR;
                reset_before_monitor = true;
            } else {
                SET_ACTION(ACT_MONITOR);
            }
        } else if (!strcmp("-r", argv[i]) || !strcmp("--reset", argv[i])) {
            if (cmd_action == ACT_MONITOR) {
                reset_before_monitor = true;
            } else {
                SET_ACTION(ACT_RESET);
            }
        } else if (!strcmp("-u", argv[i]) || !strcmp("--upload", argv[i])) {
            SET_ACTION(ACT_UPLOAD);
        } else if (!strcmp("-R", argv[i]) || !strcmp("--reset-bootsel", argv[i])) {
            SET_ACTION(ACT_RESET_BOOTSEL)
        } else if (!strcmp("-i", argv[i]) || !strcmp("--info", argv[i])) {
            SET_ACTION(ACT_INFO)
        } else if (!strcmp("-v", argv[i]) || !strcmp("--verbose", argv[i])) {
            verbose = true;
        } else if (!strcmp("-n", argv[i]) || !strcmp("--no-interaction", argv[i])) {
            no_interaction = true;
        } else if (!strcmp("-f", argv[i]) || !strcmp("--filename", argv[i])) {
            i++;
            if (i >= argc) {
                printf("Expected filename after argument\n");
                return -1;
            }
            firmware_filename = argv[i];
        } else if (!strcmp("-w", argv[i]) || !strcmp("--wait", argv[i])) {
            wait_for_connect = true;
            const char* timeout_str = argv[++i];
            if (!timeout_str) {
                printf("Expected timeout argument for wait\n");
                return -1;
            }
            for (int i = 0; i < strlen(timeout_str); i++){
                wait_timeout *= 10;
                if (timeout_str[i] < '0' || timeout_str[i] > '9') {
                    printf("Invalid timeout value: %s\n", timeout_str);
                    return -1;
                }
                wait_timeout += timeout_str[i] - '0';
            }
        } else {
            printf("Unexpected token: %s\n", argv[i]);
            return -1;
        }
    }

    // Any command specific checking
    if (cmd_action == ACT_UPLOAD && firmware_filename == NULL){
        printf("Firmware filename required!\nSpecify with --filename\n");
        return -1;
    }

    // No need for usb device for help
    if (cmd_action == ACT_HELP) {
        printf("RP2040 Programming Tool Help:\n");
        printf("%s [action] [options]\n", argv[0]);
        printf("\nActions:\n");
        printf("\t-h (--help)\tDisplays help\n");
        printf("\t-u (--uplolad) \tPrograms RP2040 with specified uf2 file. Requires --filename argument\n");
        printf("\t-m (--monitor)\tDisplays serial monitor\n");
        printf("\t-r (--reset)\tResets RP2040 into programming mode\n");
        printf("\t-R (--reset-bootsel)\tResets RP2040 into bootsel mode\n");
        printf("\t-i (--info)\tLists info on the currently connected device\n");
        printf("\nGeneral Options:\n");
        printf("\t-v (--verbose)\tEnables verbose error logging\n");
        printf("\t-n (--no-interaction)\tDisables all prompting from user\n");
        printf("\t-w (--wait) [timeout_ms]\tWait for a given timeout period for device to appear\n");
        printf("\t-f (--filename) [uf2_file]\tLoads uf2 file. Used for uploading but can also be provided to select board type\n");
        printf("\nMonitor Options:\n");
        printf("\t-r (--reset)\tResets RP2040 before opening serial port\n");
        return 0;
    } else if (cmd_action == ACT_UNSET) {
        printf("No command specified! (Usage: '%s --help')\n", argv[0]);
        return -1;
    }

    // ========================================
    // Device Loading
    // ========================================

    char board_type_mem[50];
    const char* board_type = NULL;
    if (firmware_filename != NULL) {
        if (get_uf2_board_type(firmware_filename, board_type_mem, sizeof(board_type_mem))) {
            bool board_type_found = false;
            for (int i = 0; i < num_board_types; i++){
                if (!strcmp(board_types[i].board_type_name, board_type_mem)) {
                    board_type_found = true;
                }
            }

            if (!board_type_found) {
                fprintf(stderr, "WARN: Undefined board type '%s' in uf2 file\nThis will cause issues with device auto-selection\n\n", board_type_mem);
            }

            board_type = board_type_mem;
        } else {
            fprintf(stderr, "Failed to extract board type from uf2 file\n");
            return 1;
        }
    }

    // Initialize USB
    if (libusb_init(&ctx) != LIBUSB_SUCCESS) {
        fprintf(stderr, "Failed to init libusb!");
        return 1;  // No need to cleanup
    }

    rp2040_discovery_ctx_t* device_discovery_ctx = NULL;
    rp2040_device_t *device = NULL;
    struct timeval start;
    gettimeofday(&start, NULL);
    do {
        if (device_discovery_ctx != NULL){
            cleanup_rp2040_discovery(device_discovery_ctx);
        }
        device_discovery_ctx = discover_rp2040(ctx);
        device = get_rp2040_device(device_discovery_ctx, board_type, no_interaction);
    } while (wait_for_connect && (device == NULL && get_elapsed_time(&start) < wait_timeout));

    if (device == NULL){
        if (wait_for_connect) {
            printf("Failed to select RP2040 device after %.02f seconds\n", ((float)wait_timeout)/1000.0);
        } else {
            printf("Failed to select RP2040 device\n");
        }
        goto cleanup_ctx;
    }

    // ========================================
    // Command Actions
    // ========================================

    if (cmd_action == ACT_MONITOR) {
        bool handle_okay = true;
        if (reset_before_monitor) {
            if (!rp2040_reset(device, false, false)) {
                printf("Failed to reset rp2040\n");
                ret = 1;
            } else {
                handle_okay = rediscover_rp2040_device_timeout(device, NUM_RESET_DISCOVERY_TIMEOUT_MS);
                if (!handle_okay){
                    printf("Failed to rediscover rp2040 after reset\n");
                    ret = 1;
                }
            }
        }

        if (handle_okay) {
            char portname[50];
            if (get_serial_port(device, "Board CDC", portname, sizeof(portname))) {
                serial_monitor(portname);
                ret = 0;
            } else {
                printf("No serial port found\n");
                ret = 1;
            }
        }
    } else if (cmd_action == ACT_RESET) {
        printf("Resetting RP2040...\n");
        if (rp2040_reset(device, false, false)) {
            ret = 0;
        } else {
            printf("Failed to reset rp2040\n");
            ret = 1;
        }
    } else if (cmd_action == ACT_RESET_BOOTSEL) {
        printf("Resetting RP2040 into Bootsel mode...\n");
        if (rp2040_reset(device, true, false)) {
            ret = 0;
        } else {
            printf("Failed to reset rp2040\n");
            ret = 1;
        }
    } else if (cmd_action == ACT_UPLOAD) {
        bool handle_okay = true;
        if (!device->in_bootrom_mode) {
            if (!rp2040_reset(device, true, true)) {
                printf("Failed to reset rp2040\n");
                ret = 1;
            } else {
                handle_okay = rediscover_rp2040_device_timeout(device, NUM_RESET_DISCOVERY_TIMEOUT_MS);
                if (!handle_okay){
                    printf("Failed to rediscover rp2040 after reset into bootsel\n");
                }
                ret = 1;
            }
        }

        if (handle_okay) {
            if (load_file(firmware_filename, device, true, true)) {
                ret = 0;
            } else {
                // No need to report errors
                ret = 1;
            }
        }
    } else if (cmd_action == ACT_INFO) {
        printf("Device Serial: %s\n", device->serial);
        printf("Device Board Type: %s\n", device->capabilities->board_type_name);
        printf("Device Mode: %s\n", (device->in_bootrom_mode ? "Bootrom" : "Normal"));
    }

    cleanup_rp2040_discovery(device_discovery_ctx);

cleanup_ctx:
    libusb_exit(ctx);
    return ret;


cmd_multiple:
    printf("Multiple commands specified, (Usage: '%s --help')\n", argv[0]);
    return -1;
}
