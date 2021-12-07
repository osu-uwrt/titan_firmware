#ifndef _UPLOADER_H
#define _UPLOADER_H

#include <stdbool.h>
#include <stdint.h>

#include <sys/time.h>

#include <libusb-1.0/libusb.h>

#include "picoboot_connection.h"

#ifdef __cplusplus
extern "C" {
#endif

#define RP2040_HEX_SERIAL_LEN 16

extern bool verbose;

// Definitions for board_definitions
struct rp2040_device_capabilities {
    const char* board_type_name;
    bool supports_general_code;     // If the board can run programs meant for another board (mainly dev boards like pico or eval board)
    bool supports_i2c_upload;       // If the board can be accessed via an i2c uploader
    bool supports_i2c_proxy;        // If the board can be used as a proxy from usb to i2c
};

struct rp2040_board_instance {
    const char* board_type_name;
    const char* serial;
};

extern const struct rp2040_device_capabilities board_types[];
extern const struct rp2040_device_capabilities *unknown_board_type;
extern const struct rp2040_board_instance board_definitions[];
extern const size_t num_board_types;
extern const size_t num_board_definitions;

// Device structs
typedef struct rp2040_device {
    struct rp2040_device* next;
    struct rp2040_device* prev;

    libusb_context* context;
    libusb_device_handle* handle;
    uint8_t busnum;
    uint8_t devnum;

    char serial[RP2040_HEX_SERIAL_LEN + 1];
    const struct rp2040_device_capabilities* capabilities;
    bool in_bootrom_mode;

    union {
        struct rp2040_bootrom_usb_data bootrom;
        // device struct can be added when needed
    } data;
} rp2040_device_t;

static int64_t get_elapsed_time(struct timeval* start){
    struct timeval now;
    gettimeofday(&now, NULL);
    int64_t seconds  = now.tv_sec  - start->tv_sec;
    int64_t useconds = now.tv_usec - start->tv_usec;
    int64_t ms_elapsed = (useconds / 1000) + (seconds * 1000);
    return ms_elapsed;
}

typedef struct rp2040_device rp2040_discovery_ctx_t;

// Discover
bool get_serial_port(rp2040_device_t* device, const char* target_interface_name, char* port_out, size_t portout_length);
rp2040_discovery_ctx_t* discover_rp2040(libusb_context *ctx);
void cleanup_rp2040_discovery(rp2040_device_t* device);
rp2040_device_t* get_rp2040_device(rp2040_discovery_ctx_t* ctx, const char* target_type, bool no_interaction);
bool rediscover_rp2040_device_timeout(rp2040_device_t* device, unsigned int ms);

// Device Utility Functions
bool rp2040_reset(rp2040_device_t* dev_handle, bool bootsel, bool picoboot_only);
void serial_monitor(char* portname);

// Picotool imported functions
bool get_uf2_board_type(const char* filename, char* name_out, size_t name_size);
bool load_file(const char* filename, rp2040_device_t* device, bool verify, bool execute_after);
bool reboot_bootsel_to_bootsel(rp2040_device_t* device, uint32_t disable_interface_mask);

#ifdef __cplusplus
}
#endif

#endif // _UPLOADER_H