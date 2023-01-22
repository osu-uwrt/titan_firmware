#include <stdio.h>
#include "pico/binary_info.h"
#include "pico/stdlib.h"

#include <uxr/client/profile/transport/custom/custom_transport.h>
#include <rmw_microros/rmw_microros.h>

#include "can_pio/canbus.h"
#include "hardware/clocks.h"
#include "hardware/irq.h"

#include "micro_ros_pico/transport_can.h"

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

bool transport_can_open(__unused struct uxrCustomTransport * transport)
{
    return canbus_initialized;
}

bool transport_can_close(__unused struct uxrCustomTransport * transport)
{
    return true;
}

size_t transport_can_write(__unused struct uxrCustomTransport* transport, const uint8_t *buf, size_t len, uint8_t *errcode)
{
    while (!canbus_msg_write_available()) {
        tight_loop_contents();
    }

    return canbus_msg_write(buf, len);
}

size_t transport_can_read(__unused struct uxrCustomTransport * transport, uint8_t *buf, size_t len, int timeout, uint8_t *errcode)
{
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

void transport_can_init(uint bitrate,
                             uint client_id,
                             spi_inst_t* spi_channel,
                             uint8_t cs_pin,
                             uint8_t mosi_pin,
                             uint8_t miso_pin,
                             uint8_t sck_pin,
                             uint32_t spi_clock,
                             uint8_t int_pin){
    canbus_init(bitrate, client_id, spi_channel, cs_pin, mosi_pin, miso_pin, sck_pin, spi_clock, int_pin);

    rmw_uros_set_custom_transport(
		false,
		NULL,
		transport_can_open,
		transport_can_close,
		transport_can_write,
		transport_can_read
	);
}