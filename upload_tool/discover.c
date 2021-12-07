#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <termios.h>
#include <unistd.h>
#include <libudev.h>
#include <libusb-1.0/libusb.h>

#include "uploader.h"
#include "read_flash.h"

#define DISCOVERY_POLL_RATE_MS 50

#define PICO_VID 0x2e8a
#define PICO_BOOTROM_PID 0x0003
#define PICO_SERIAL_PID 0x000a

static const struct rp2040_device_capabilities* lookup_board_capabilities(const char* serial){
    const char* board_name = NULL;
    for (int i = 0; i < num_board_definitions; i++){
        if (!strcmp(board_definitions[i].serial, serial)){
            board_name = board_definitions[i].board_type_name;
        }
    }

    if (board_name == NULL){
        if (verbose)
            fprintf(stderr, "Failed to locate serial number '%s' in board_definitions.c\n", serial);
        return unknown_board_type;
    }

    for (int i = 0; i < num_board_types; i++){
        if (!strcmp(board_types[i].board_type_name, board_name)){
            return &board_types[i];
        }
    }

    fprintf(stderr, "WARNING: Board type %s defined for serial '%s' but not actually defined as a board capability in board_definitions.c\n", board_name, serial);
    return NULL;
}

static bool setup_and_claim_bootrom_interface(libusb_device_handle *handle, struct rp2040_bootrom_usb_data *usbdata) {
    struct libusb_config_descriptor *config;
    int err = libusb_get_active_config_descriptor(libusb_get_device(handle), &config);

    unsigned int interface = 0, out_ep = 0, in_ep = 0;

    if (err){
        if (verbose) 
            fprintf(stderr, "Failed to read configuration descriptor (%d: %s)\n", err, libusb_error_name(err));
        return false;
    }

    if (config->bNumInterfaces == 1) {
        interface = 0;
    } else {
        interface = 1;
    }
    if (config->interface[interface].altsetting[0].bInterfaceClass == 0xff &&
        config->interface[interface].altsetting[0].bNumEndpoints == 2) {
        out_ep = config->interface[interface].altsetting[0].endpoint[0].bEndpointAddress;
        in_ep = config->interface[interface].altsetting[0].endpoint[1].bEndpointAddress;
    } else {
        if (verbose)
            fprintf(stderr, "Error: Interface %u is not valid picoboot interface\n", interface);
        return false;
    }

    if (!out_ep || !in_ep || (out_ep & 0x80u) || !(in_ep & 0x80u)) {
        if (verbose)
            fprintf(stderr, "Invalid endpoints (in: %u, out: %u) for picoboot interface (interface: %u)\n", in_ep, out_ep, interface);
        return false;   
    }

    err = libusb_claim_interface(handle, interface);
    if (err) {
        if (verbose) 
            fprintf(stderr, "Failed to claim interface (%d: %s)\n", err, libusb_error_name(err));
        return false;
    }

    usbdata->interface = interface;
    usbdata->out_ep = out_ep;
    usbdata->in_ep = in_ep;

    return true;
}

static bool lookup_bootrom_serial(libusb_device_handle *handle, char* serial_output){
    // Buffer must be at least RP2040_HEX_SERIAL_LEN + 1 size

    struct rp2040_bootrom_usb_data usbdata;
    if (!setup_and_claim_bootrom_interface(handle, &usbdata)){
        if (verbose)
            fprintf(stderr, "Could not lookup bootrom serial number: Could not claim picoboot interface\n");
        
        return false;
    }

    // Get exclusive access... We don't really care to restore it since most commands using this should 
    int err = picoboot_exclusive_access(handle, &usbdata, 2);
    if (err) {
        if (verbose)
            fprintf(stderr, "Failed to set picoboot exclusive mode (%d)\n", err);
        goto close_interface;
    }

    bool serial_valid = false;

    char temp_serial_output[RP2040_HEX_SERIAL_LEN + 1] = {0};
    uint8_t serial[8] = {0};

    err = picoboot_write(handle, &usbdata, 0x20001000, serial, sizeof(serial));
    if (err){
        if (verbose)
            fprintf(stderr, "Failed to clear serial memory location (%d)\n", err);
        goto close_interface;
    }

    err = picoboot_exit_xip(handle, &usbdata);
    if (err){
        if (verbose)
            fprintf(stderr, "Failed to exit xip (%d)\n", err);
        goto close_interface;
    }

    err = picoboot_write(handle, &usbdata, 0x20000000, flash_bin, flash_bin_len);
    if (err){
        if (verbose)
            fprintf(stderr, "Failed to write serial read executable (%d)\n", err);
        goto close_interface;
    }

    err = picoboot_exec(handle, &usbdata, 0x20000000);
    if (err){
        if (verbose)
            fprintf(stderr, "Failed to execute serial read (%d)\n", err);
        goto close_interface;
    }

    err = picoboot_enter_cmd_xip(handle, &usbdata);
    if (err){
        if (verbose)
            fprintf(stderr, "Failed to re-enter xip (%d)\n", err);
        goto close_interface;
    }

    err = picoboot_read(handle, &usbdata, 0x20001000, serial, sizeof(serial));
    if (err){
        if (verbose)
            fprintf(stderr, "Failed to read out serial memory location (%d)\n", err);
        goto close_interface;
    }

    err = picoboot_exclusive_access(handle, &usbdata, 0);
    if (err){
        if (verbose)
            fprintf(stderr, "Failed to remove picoboot exclusive access (%d)\n", err);
        goto close_interface;
    }

    for (int i = 0; i < sizeof(serial); i++){
        if (serial[i] != 0 && serial[i] != 0xaa) {  // Libusb returns all 0xaa when it breaks
            serial_valid = true;
        }
        sprintf(&temp_serial_output[i*2], "%02X", serial[i]);
    }
    if (serial_valid){
        strcpy(serial_output, temp_serial_output);
    } else if (verbose) {
        fprintf(stderr, "Could not lookup bootrom serial number: Invalid serial number received from device\n");
    }

close_interface:
    libusb_release_interface(handle, usbdata.interface);
    return serial_valid;
}

static bool enumerate_rp2040(libusb_device_handle *handle, struct libusb_device_descriptor* desc, rp2040_device_t *device){
    // Lookup serial number
    char serial_number[RP2040_HEX_SERIAL_LEN + 2];  // Give 1 extra character to see if the serial is too long
    if (desc->idProduct == PICO_BOOTROM_PID) {
        if (!lookup_bootrom_serial(handle, serial_number)){
            return false;
        }
    } else {
        int err = libusb_get_string_descriptor_ascii(handle, desc->iSerialNumber, serial_number, sizeof(serial_number));
        if (err < 0) {
            if (verbose)
                fprintf(stderr, "Failed to read usb string descriptor\n");
            return false;
        }

        if (err != RP2040_HEX_SERIAL_LEN) {
            if (verbose)
                fprintf(stderr, "Unexpected serial number '%s'\n", serial_number);
            return false;
        }
    }

    // Get device capabilities from serial number
    const struct rp2040_device_capabilities* board_capabilities = lookup_board_capabilities(serial_number);
    if (board_capabilities == NULL) {
        return false;
    }

    // Fill the new device struct
    device->handle = handle;
    device->busnum = libusb_get_bus_number(libusb_get_device(device->handle));
    device->devnum = libusb_get_device_address(libusb_get_device(device->handle));

    strncpy(device->serial, serial_number, RP2040_HEX_SERIAL_LEN + 1);
    device->capabilities = board_capabilities;
    device->in_bootrom_mode = (desc->idProduct == PICO_BOOTROM_PID);

    if (device->in_bootrom_mode){
        if (!setup_and_claim_bootrom_interface(handle, &device->data.bootrom)){
            return false;
        }
    }

    return true;
}

rp2040_discovery_ctx_t* discover_rp2040(libusb_context *ctx){
    struct rp2040_device* context = malloc(sizeof(struct rp2040_device));
    context->next = NULL;
    context->prev = NULL;
    struct rp2040_device* front_device = context;
    
    libusb_device **list;
    ssize_t cnt = libusb_get_device_list(ctx, &list);
    ssize_t i = 0;

    int err = 0;
    if (cnt < 0) {
        if (verbose)
            fprintf(stderr, "Error getting libusb device list!\n");
        return context;
    }
    
    for (i = 0; i < cnt; i++) {
        libusb_device *usb_device = list[i];
        struct libusb_device_descriptor desc;
        
		if (libusb_get_device_descriptor(usb_device, &desc) < 0) {
            if (verbose)
                fprintf(stderr, "Failed to read device descriptor!\n");
            continue;
        }
		
        if (desc.idVendor == PICO_VID && (desc.idProduct == PICO_SERIAL_PID || desc.idProduct == PICO_BOOTROM_PID)) {
            libusb_device_handle *handle = NULL;
            err = libusb_open(usb_device, &handle);
            if (err) {
                if (verbose)
                    fprintf(stderr, "Failed to open libusb device (check permissions?)\n");
                continue;
            }

            struct rp2040_device* device = malloc(sizeof(struct rp2040_device));
            device->context = ctx;

            // Enumerate the device struct
            if (!enumerate_rp2040(handle, &desc, device)) {
                libusb_close(handle);
                free(device);
                continue;
            }

            // Add to linked list
            front_device->next = device;
            device->prev = front_device;
            device->next = NULL;
            front_device = device;
        }
    }

    libusb_free_device_list(list, 1);

    return context;
}

rp2040_device_t* get_rp2040_device(rp2040_discovery_ctx_t* ctx, const char* target_type, bool no_interaction){
    if (ctx->next == NULL){
        return NULL;
    }

    // First try to find device of target_type without input from user
    rp2040_device_t* entry = ctx->next;
    rp2040_device_t* device = NULL;
    int device_count = 0;
    bool duplicate = false;
    while (entry != NULL){
        device_count++;
        if ((target_type == NULL || !strcmp(entry->capabilities->board_type_name, target_type)) && !duplicate) {
            if (device != NULL){
                // Duplicate found, input needed from user
                device = NULL;
                duplicate = true;
            } else {
                device = entry;
            }
        }
        entry = entry->next;
    }

    if (device != NULL){
        return device;
    }

    // If no_interaction, then break here, there's nothing that can be done
    if (no_interaction) {
        return NULL;
    }

    // No device matching target_type found, or there were duplicates. Now request input from user to select board type
    printf("=======Select Target Device=======\n");

    rp2040_device_t* menu_ordering[device_count];
    int device_index = 0;

    // First try to print matching devices
    bool header_printed = false;
    entry = ctx->next;
    while (entry != NULL) {
        if (target_type != NULL && !strcmp(entry->capabilities->board_type_name, target_type)) {
            if (!header_printed){
                header_printed = true;
                printf("%s Boards:\n", entry->capabilities->board_type_name);
            }
            menu_ordering[device_index] = entry;
            printf("%d: %s Mode (%s)\n", device_index, (entry->in_bootrom_mode ? "Bootrom" : "Normal"), entry->serial);
            device_index++;
        }
        entry = entry->next;
    }
    if (header_printed){
        printf("\n");
    }

    // Then print non-matching devices
    header_printed = false;
    entry = ctx->next;
    while (entry != NULL) {
        if (target_type == NULL || strcmp(entry->capabilities->board_type_name, target_type)) {
            if (!header_printed){
                header_printed = true;
                if (target_type != NULL) {
                    printf("Other Boards:\n");
                }
            }
            menu_ordering[device_index] = entry;
            printf("%d: %s - %s Mode (%s)\n", device_index, entry->capabilities->board_type_name, (entry->in_bootrom_mode ? "Bootrom" : "Normal"), entry->serial);
            device_index++;
        }
        entry = entry->next;
    }
    if (header_printed){
        printf("\n");
    }

    assert(device_index == device_count);

    // Get selection from user
    while (device == NULL) {
        printf("Select Device: ");

        int num, nitems;
        nitems = scanf("%d", &num);
        if (nitems == EOF) {
            return NULL;
        } else if (nitems == 1) {
            while ( getchar() != '\n' );
            if (num >= 0 && num < device_index){
                device = menu_ordering[num];
            } else {
                printf("Invalid Index!\n\n");
            }
        } else {
            while ( getchar() != '\n' );
            printf("Invalid Input!\n\n");
        }
    }

    if (target_type != NULL && !device->capabilities->supports_general_code && strcmp(device->capabilities->board_type_name, target_type)) {
        printf("\nWARNING: This board is marked as not supporting general code!\nEnsure that this operation will work on this board and not damage any surrounding circuitry.\nAre you sure that you want to continue? [y/N]: ");

        struct termios old_tio, new_tio;
        tcgetattr(STDIN_FILENO,&old_tio);
        new_tio=old_tio;
        new_tio.c_lflag &=(~ICANON);
        tcsetattr(STDIN_FILENO,TCSANOW,&new_tio);

        unsigned char c = getchar();
        if (c != '\n') printf("\n");
        if (c != 'y' && c != 'Y') {
            printf("Aborted\n");
            device = NULL;
        }

        tcsetattr(STDIN_FILENO,TCSANOW,&old_tio);
    }

    printf("\n");

    return device;
}

void cleanup_rp2040_discovery(rp2040_discovery_ctx_t* ctx) {
    struct rp2040_device* node = ctx->next;
    free(ctx);   // rp2040_discvoery_ctx_t is a dummy node, no need to clean up

    while (node != NULL) {
        struct rp2040_device* device = node;
        node = node->next;

        if (device->in_bootrom_mode) {
            libusb_release_interface(device->handle, device->data.bootrom.interface);
        }

        libusb_close(device->handle);
        free(device);
    }

}

bool rediscover_rp2040_device(rp2040_device_t* device){
    // Note: If this function returns false, then the device has been freed and is no longer valid
    // Returns 0 on success, -1 on not found (still resetting), 1 on free (like enumeration failure, or when free_on_nondiscovery is set)
    libusb_device **list;
    libusb_context* ctx = device->context;

    size_t cnt = libusb_get_device_list(ctx, &list);
    ssize_t i = 0;
    
    int err = 0;
    if (cnt < 0) {
        if (verbose)
            fprintf(stderr, "Error getting libusb device list!\n");
        return false;
    }
    
    bool found = false;

    for (i = 0; i < cnt; i++) {
        libusb_device *usb_device = list[i];

        struct libusb_device_descriptor desc;
        if (libusb_get_device_descriptor(usb_device, &desc) < 0) {
            if (verbose)
                fprintf(stderr, "Failed to re-read device descriptor!\n");
            continue;
        }

        if (libusb_get_bus_number(usb_device) == device->busnum && desc.idVendor == PICO_VID && (desc.idProduct == PICO_SERIAL_PID || desc.idProduct == PICO_BOOTROM_PID)) {
            uint8_t busnum = libusb_get_bus_number(usb_device);
            uint8_t devnum = libusb_get_device_address(usb_device);


            // Make sure the device doesn't exist already
            bool device_exists = false;
            rp2040_device_t* search_target = device->prev;
            while (!device_exists && search_target->prev != NULL){
                if (search_target->busnum == busnum && search_target->devnum == devnum) {
                    device_exists = true;
                }
                search_target = search_target->prev;
            }
            search_target = device->next;
            while (!device_exists && search_target != NULL){
                if (search_target->busnum == busnum && search_target->devnum == devnum) {
                    device_exists = true;
                }
                search_target = search_target->next;
            }
            if (device_exists) {
                continue;
            }

            libusb_device_handle *handle = NULL;
            err = libusb_open(usb_device, &handle);
            if (err) {
                if (verbose)
                    fprintf(stderr, "Failed to open libusb device during re-discovery (check permissions?) [%d: %s]\n", err, libusb_strerror(err));
                continue;
            }

            device->in_bootrom_mode = (desc.idProduct == PICO_BOOTROM_PID);

            // Lookup serial number
            char serial_number_check[RP2040_HEX_SERIAL_LEN + 2];  // Give 1 extra character to see if the serial is too long
            if (desc.idProduct == PICO_BOOTROM_PID) {
                if (!lookup_bootrom_serial(handle, serial_number_check)){
                    libusb_close(handle);
                    continue;
                }
            } else {
                err = libusb_get_string_descriptor_ascii(handle, desc.iSerialNumber, serial_number_check, sizeof(serial_number_check));
                if (err < 0) {
                    if (verbose)
                        fprintf(stderr, "Failed to read usb string descriptor for serial number during re-discovery\n");
                    libusb_close(handle);
                    continue;
                }

                if (err != RP2040_HEX_SERIAL_LEN) {
                    if (verbose)
                        fprintf(stderr, "Unexpected serial number '%s' during re-discovery\n", serial_number_check);
                    libusb_close(handle);
                    continue;
                }
            }

            if (strcmp(device->serial, serial_number_check)){
                libusb_close(handle);
                continue;
            }

            // We now know that they are the same device

            if (device->in_bootrom_mode){
                if (!setup_and_claim_bootrom_interface(handle, &device->data.bootrom)){
                    libusb_close(handle);
                    continue;
                }
            }

            device->busnum = busnum;
            device->devnum = devnum;
            device->handle = handle;
            found = true;
        }
    }

    libusb_free_device_list(list, 1);

    return found;
}

bool rediscover_rp2040_device_timeout(rp2040_device_t* device, unsigned int ms) {
    // Note: Frees device if false is returned

    // Free all resources from the device
    if (device->in_bootrom_mode) {
        libusb_release_interface(device->handle, device->data.bootrom.interface);
    }
    libusb_close(device->handle);

    struct timeval start;
    gettimeofday(&start, NULL);
    bool found = false;

    do {
        usleep(DISCOVERY_POLL_RATE_MS * 1000);
        found = rediscover_rp2040_device(device);
    } while (!found && get_elapsed_time(&start) < ms);


    if (!found) {
        // Should always have prev because of dummy node
        device->prev->next = device->next;
        if (device->next)
            device->next->prev = device->prev;
        free(device);

        if (verbose)
            fprintf(stderr, "Could not find rp2040 after timeout\n");
    }

    // Extra delay to make sure udev picks up the device
    usleep(50000);

    return found;
}

bool get_serial_port(rp2040_device_t* device, const char* target_interface_name, char* port_out, size_t portout_length) {
    /* Taken from udev_example2.c
    *
    * Copyright (C) 2021 Robert Pafford
    * Copyright (C) 2014 Robert Milasan <rmilasan@suse.com>
    *
    * This program is free software: you can redistribute it and/or modify
    * it under the terms of the GNU General Public License as published by
    * the Free Software Foundation, either version 2 of the License, or
    * (at your option) any later version.
    *
    * This program is distributed in the hope that it will be useful,
    * but WITHOUT ANY WARRANTY; without even the implied warranty of
    * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    * GNU General Public License for more details.
    *
    * You should have received a copy of the GNU General Public License
    * along with this program.  If not, see <http://www.gnu.org/licenses/>.
    *
    * This example will enumerate all devices belonging to 'block' subsystem 
    * and omit partitions and loop devices using libudev API.
    *
    */
    bool port_found = false;

    if (device->in_bootrom_mode) {
        if (verbose)
            fprintf(stderr, "Cannot load serial port of device in bootrom mode\n");
        return false;
    }

    uint8_t target_busnum = device->busnum;
    uint8_t target_devnum = device->devnum;

    char busstr[4];
    char devstr[4];
    snprintf(busstr, sizeof(busstr), "%03d", target_busnum);
    snprintf(devstr, sizeof(devstr), "%03d", target_devnum);

    struct udev *udev;
	struct udev_device *dev;
	struct udev_enumerate *enumerate;
	struct udev_list_entry *devices, *device_list_entry, *ttys, *tty_list_entry;

    /*
     * Find the usb device by bus and device number
     */

	/* create udev object */
	udev = udev_new();
	if (!udev) {
        if (verbose)
		    fprintf(stderr, "Cannot create udev context.\n");
		goto return_now;
	}

	/* create enumerate object */
	enumerate = udev_enumerate_new(udev);
	if (!enumerate) {
        if (verbose)
		    fprintf(stderr, "Cannot create enumerate context.\n");
		goto udev_cleanup;
	}

	udev_enumerate_add_match_subsystem(enumerate, "usb");
    udev_enumerate_add_match_property(enumerate, "BUSNUM", busstr);
	udev_enumerate_scan_devices(enumerate);

	/* fillup device list */
	devices = udev_enumerate_get_list_entry(enumerate);
	if (!devices) {
        if (verbose)
		    fprintf(stderr, "Failed to get device list.\n");
		udev_enumerate_unref(enumerate);
        goto udev_cleanup;
	}

    bool found = false;

    udev_list_entry_foreach(device_list_entry, devices) {
        const char* dev_path = udev_list_entry_get_name(device_list_entry);
	    dev = udev_device_new_from_syspath(udev, dev_path);

        const char* devnum = udev_device_get_property_value(dev, "DEVNUM");
        if (!strcmp(devnum, devstr)){
            // Break and don't free dev so it can be used later
            found = true;
            break;
        }

        /* free usb device */
        udev_device_unref(dev);
    }

    /* free enumerate */
	udev_enumerate_unref(enumerate);

    if (!found){
        if (verbose)
            fprintf(stderr, "Failed to get device list.\n");
		goto udev_cleanup;
    }

    /*
     * Get the requested serial port
     */

    /* create enumerate object */
	enumerate = udev_enumerate_new(udev);
	if (!enumerate) {
        if (verbose)
		    fprintf(stderr, "Cannot create enumerate context.\n");
		goto usbdev_cleanup;
	}

	udev_enumerate_add_match_subsystem(enumerate, "tty");
    udev_enumerate_add_match_parent(enumerate, dev);
	udev_enumerate_scan_devices(enumerate);

	/* fillup device list */
	ttys = udev_enumerate_get_list_entry(enumerate);
    if (!ttys) {
        if (verbose)
		    fprintf(stderr, "Failed to get tty list.\n");
		goto ttyenum_cleanup;
	}

    udev_list_entry_foreach(tty_list_entry, ttys) {
        const char* path = udev_list_entry_get_name(tty_list_entry);
        struct udev_device *tty = udev_device_new_from_syspath(udev, path);

        const char* devnode = udev_device_get_devnode(tty);
        const char* interface_name = udev_device_get_sysattr_value(udev_device_get_parent(tty), "interface");

        if (target_interface_name == NULL || (interface_name != NULL && !strcmp(interface_name, target_interface_name))) {
            if (port_found) {
                if (verbose)
                    fprintf(stderr, "Multiple serial interfaces found, returning none.\n");
                port_found = false;
                port_out[0] = '\0';
                udev_device_unref(tty);
                break;
            }

            if (strlen(devnode) < portout_length){
                strncpy(port_out, devnode, portout_length);
                port_found = true;
            }
        }

        udev_device_unref(tty);
    }

ttyenum_cleanup:
    /* free tty enumeration */
    udev_enumerate_unref(enumerate);

usbdev_cleanup:
    /* free usb device */
    udev_device_unref(dev);
udev_cleanup:
	/* free udev */
	udev_unref(udev);

return_now:
    return port_found;
}