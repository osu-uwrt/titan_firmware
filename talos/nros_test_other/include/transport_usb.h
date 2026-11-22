#ifndef TRANSPORT_USB_H
#define TRANSPORT_USB_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

typedef struct {
    uint8_t err_code;
    int32_t timeout;
} transport_data_t;

bool transport_usb_open();
bool transport_usb_close();
size_t transport_usb_write(const uint8_t *buf, size_t len, void *user_data);
size_t transport_usb_read(uint8_t *buf, size_t len, void *user_data);
bool transport_usb_init(void *ctx);

#endif TRANSPORT_USB_H