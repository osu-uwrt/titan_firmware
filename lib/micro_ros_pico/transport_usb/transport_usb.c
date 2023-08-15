#include "micro_ros_pico/transport_usb.h"

#include "pico/binary_info.h"
#include "pico/stdio_usb.h"
#include "pico/time.h"

#include <rmw_microros/rmw_microros.h>
#include <uxr/client/profile/transport/custom/custom_transport.h>

#include <stdint.h>

static bool transport_initialized = false;
void transport_usb_serial_init_early(void) {
    if (!transport_initialized) {
        stdio_init_all();
        dual_usb_init();
        transport_initialized = true;
    }
}

bool transport_usb_open(__unused struct uxrCustomTransport *transport) {
    transport_usb_serial_init_early();
    return true;
}

bool transport_usb_close(__unused struct uxrCustomTransport *transport) {
    return true;
}

size_t transport_usb_write(__unused struct uxrCustomTransport *transport, const uint8_t *buf, size_t len,
                           uint8_t *errcode) {
    size_t sent = secondary_usb_out_chars(buf, len);
    if (sent != len) {
        *errcode = 1;
    }
    return sent;
}

size_t transport_usb_read(__unused struct uxrCustomTransport *transport, uint8_t *buf, size_t len, int timeout,
                          uint8_t *errcode) {
    int64_t start_time_us = time_us_64();
    int64_t elapsed_time_us = timeout * 1000 - (time_us_64() - start_time_us);
    size_t bytes_remaining = len;
    while (bytes_remaining > 0 && elapsed_time_us > 0) {
        int received = secondary_usb_in_chars(buf, bytes_remaining);
        if (received > 0) {
            bytes_remaining -= received;
            // TODO: Figure out why incrementing buf by # received chars breaks stuff
        }
        else {
            busy_wait_us(1);
        }
        elapsed_time_us = timeout * 1000 - (time_us_64() - start_time_us);
    }

    if (bytes_remaining > 0) {
        *errcode = 1;
    }
    return (len - bytes_remaining);
}

bi_decl(bi_program_feature("Micro-ROS over USB"))

    void transport_usb_init(void) {
    rmw_uros_set_custom_transport(true, NULL, transport_usb_open, transport_usb_close, transport_usb_write,
                                  transport_usb_read);
}
