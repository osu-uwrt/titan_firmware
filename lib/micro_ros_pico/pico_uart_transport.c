#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/stdio_usb.h"

#include <uxr/client/profile/transport/custom/custom_transport.h>
#include <rmw_microros/rmw_microros.h>

void usleep(uint64_t us)
{
    sleep_us(us);
}

int clock_gettime(clockid_t unused, struct timespec *tp)
{
    uint64_t m = time_us_64();
    tp->tv_sec = m / 1000000;
    tp->tv_nsec = (m % 1000000) * 1000;
    return 0;
}

void serial_init_early(void)
{
    stdio_init_all();
    dual_usb_init();
}

bool pico_serial_transport_open(struct uxrCustomTransport * transport)
{
    stdio_init_all();
    dual_usb_init();
    return true;
}

bool pico_serial_transport_close(struct uxrCustomTransport * transport)
{
    return true;
}

size_t pico_serial_transport_write(struct uxrCustomTransport* transport, const uint8_t *buf, size_t len, uint8_t *errcode)
{
    int sent = secondary_usb_out_chars(buf, len);
    if (sent != len){
        *errcode = 1;
    }
    return sent;
}

size_t pico_serial_transport_read(struct uxrCustomTransport * transport, uint8_t *buf, size_t len, int timeout, uint8_t *errcode)
{
    int64_t start_time_us = time_us_64();
    int64_t elapsed_time_us = timeout * 1000 - (time_us_64() - start_time_us);
    size_t bytes_remaining = len;
    while (bytes_remaining > 0 && elapsed_time_us > 0) {
        int received = secondary_usb_in_chars(buf, bytes_remaining);
        if (received > 0) {
            bytes_remaining -= received;
        } else {
            busy_wait_us(1);
        }
        elapsed_time_us = timeout * 1000 - (time_us_64() - start_time_us);
    }

    if (bytes_remaining > 0) {
        *errcode = 1;
    }
    //printf("Read %d bytes from serial...\n", (len - bytes_remaining));
    return (len - bytes_remaining);
}

void pico_serial_transport_init(void){
    rmw_uros_set_custom_transport(
		true,
		NULL,
		pico_serial_transport_open,
		pico_serial_transport_close,
		pico_serial_transport_write,
		pico_serial_transport_read
	);
}