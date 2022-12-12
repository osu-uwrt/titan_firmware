#include <stdio.h>
#include "pico/binary_info.h"
#include "pico/stdlib.h"

#include <uxr/client/profile/transport/custom/custom_transport.h>
#include <rmw_microros/rmw_microros.h>

#include "can_pio/canbus.h"
#include "hardware/clocks.h"
#include "hardware/irq.h"

void usleep(uint64_t us)
{
    sleep_us(us);
}

int clock_gettime(__unused clockid_t unused, struct timespec *tp)
{
    uint64_t m = time_us_64();
    tp->tv_sec = m / 1000000;
    tp->tv_nsec = (m % 1000000) * 1000;
    return 0;
}

bool pico_can_transport_open(__unused struct uxrCustomTransport * transport)
{
    return canbus_initialized;
}

bool pico_can_transport_close(__unused struct uxrCustomTransport * transport)
{
    return true;
}

size_t pico_can_transport_write(__unused struct uxrCustomTransport* transport, const uint8_t *buf, size_t len, uint8_t *errcode)
{
    while (!canbus_msg_write_available()) {
        tight_loop_contents();
    }

    return canbus_msg_write(buf, len);
}

size_t pico_can_transport_read(__unused struct uxrCustomTransport * transport, uint8_t *buf, size_t len, int timeout, uint8_t *errcode)
{
    // TODO: Figure what units timeout uses
    absolute_time_t timeout_time = make_timeout_time_ms(timeout);
    while (!canbus_msg_read_available()) {
        if (time_reached(timeout_time)) {
            // TODO: Should errcode be set?
            return 0;
        }
    }

    return canbus_msg_read(buf, len);
}

bi_decl(bi_program_feature("Micro-ROS over CAN"))

void pico_can_transport_init(unsigned pio_num, unsigned bitrate, unsigned client_id, unsigned gpio_rx, unsigned gpio_tx, int gpio_term){
    canbus_init(pio_num, bitrate, client_id, gpio_rx, gpio_tx, gpio_term);

    rmw_uros_set_custom_transport(
		false,
		NULL,
		pico_can_transport_open,
		pico_can_transport_close,
		pico_can_transport_write,
		pico_can_transport_read
	);
}