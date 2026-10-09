#include "transport_usb.h"

#include "pico/binary_info.h"
#include "pico/stdio_usb.h"
#include "pico/time.h"

#include <stdint.h>

static bool transport_initialized = false;

void transport_usb_serial_init_early(void) {
    if (!transport_initialized) {
        stdio_init_all();
        dual_usb_init();
        transport_initialized = true;
    }
}

bool transport_usb_open() {
    transport_usb_serial_init_early();
    return true;
}

bool transport_usb_close() {
    return true;
}

size_t transport_usb_write(const uint8_t *buf, size_t len, void *user_ctx) {
    transport_data_t *ctx = (transport_data_t *) user_ctx;
    size_t sent = secondary_usb_out_chars(buf, len);
    if (sent != len) {
        ctx->err_code = 1;
    }
    return sent;
}

// size_t transport_usb_read(uint8_t *buf, size_t len, void *user_ctx) {
//     transport_data_t *ctx = (transport_data_t *) user_ctx;
//     int64_t start_time_us = time_us_64();
//     int64_t elapsed_time_us = ctx->timeout * 1000 - (time_us_64() - start_time_us);
//     size_t bytes_remaining = len;
//     while (bytes_remaining > 0 && elapsed_time_us > 0) {
//         int received = secondary_usb_in_chars(buf, bytes_remaining);
//         if (received > 0) {
//             bytes_remaining -= received;
//             // printf("received %d bytes\n", received);
//         }
//         else {
//             busy_wait_us(1);
//         }
//         elapsed_time_us = ctx->timeout * 1000 - (time_us_64() - start_time_us);
//     }

//     if (bytes_remaining > 0) {
//         ctx->err_code = 1;
//     }
//     return (len - bytes_remaining);
// }

size_t transport_usb_read(uint8_t *buf, size_t len, void *user_ctx) {
    transport_data_t *ctx = (transport_data_t *) user_ctx;
    int64_t start_time_us = time_us_64();
    int64_t elapsed_time_us = ctx->timeout * 1000 - (time_us_64() - start_time_us);
    size_t bytes_have = 0;
    while (bytes_have < len && elapsed_time_us > 0) {
        int received = secondary_usb_in_chars(buf + bytes_have, len - bytes_have);
        if (received > 0) {
            bytes_have += received;
        } else {
            busy_wait_us(1);
        }
        elapsed_time_us = ctx->timeout * 1000 - (time_us_64() - start_time_us);
    }

    if (bytes_have < len) {
        ctx->err_code = 1;
    }
    return bytes_have;
}

bool transport_usb_init(void *ctx) {
    return true;
}

bi_decl(bi_program_feature("Nano Ros over dual USB"));
