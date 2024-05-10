#include "micro_ros_pico/transport_can.h"

#include "driver/canbus.h"
#include "pico/binary_info.h"
#include "pico/time.h"

#include <rmw_microros/rmw_microros.h>
#include <uxr/client/profile/transport/custom/custom_transport.h>

bool transport_can_open(__unused struct uxrCustomTransport *transport) {
    return canbus_msg_open();
}

bool transport_can_close(__unused struct uxrCustomTransport *transport) {
    canbus_msg_close();
    return true;
}

size_t transport_can_write(__unused struct uxrCustomTransport *transport, const uint8_t *buf, size_t len,
                           __unused uint8_t *errcode) {
    // Preents write latencies from stacking up
    // If the bus hangs, the write buffer will fill up, and if we have multiple writes in a row, we can
    // time out the watchdog. This ensures that after the timeout fires, we will immediately fail until
    // we can write the next byte, then the timeouts get re-enabled
    static bool last_write_timed_out = false;

    // Setting up a timeout to avoid crashing
    absolute_time_t timeout = make_timeout_time_ms(RMW_UXRCE_PUBLISH_RELIABLE_TIMEOUT);

    while (!canbus_msg_write_available()) {
        if (time_reached(timeout) || last_write_timed_out) {
            last_write_timed_out = true;
            *errcode = 1;
            return 0;
        }
        tight_loop_contents();
    }

    last_write_timed_out = false;
    size_t tx_len = canbus_msg_write(buf, len);

    // As we're in packet-oriented mode, we should set errcode if not all the bytes are sent
    if (len != tx_len) {
        *errcode = 2;
    }

    return tx_len;
}

size_t transport_can_read(__unused struct uxrCustomTransport *transport, uint8_t *buf, size_t len, int timeout,
                          __unused uint8_t *errcode) {
    absolute_time_t timeout_time = make_timeout_time_ms(timeout);
    while (!canbus_msg_read_available()) {
        if (time_reached(timeout_time)) {
            *errcode = 1;
            return 0;
        }
    }

    size_t rx_len = canbus_msg_read(buf, len);

    // As we're in packet-oriented mode, we must set errcode if not all the bytes are received
    if (len != rx_len) {
        *errcode = 2;
    }

    return rx_len;
}

bi_decl(bi_program_feature("Micro-ROS over CAN"));

bool transport_can_init(uint8_t client_id) {
    if (!canbus_init(client_id)) {
        return false;
    }

    if (rmw_uros_set_custom_transport(false, NULL, transport_can_open, transport_can_close, transport_can_write,
                                      transport_can_read) != RMW_RET_OK) {
        return false;
    }

    return true;
}
