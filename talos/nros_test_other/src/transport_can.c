#include "transport_can.h"

#include "driver/canbus.h"
#include "pico/binary_info.h"
#include "pico/time.h"

can_transport_data_t *t = { 0 };

bool transport_can_open() {
    return canbus_msg_open();
}

bool transport_can_close() {
    canbus_msg_close();
    return true;
}

size_t transport_can_write(const uint8_t *buf, size_t len, void *ctx) {
    // Preents write latencies from stacking up
    // If the bus hangs, the write buffer will fill up, and if we have multiple writes in a row, we can
    // time out the watchdog. This ensures that after the timeout fires, we will immediately fail until
    // we can write the next byte, then the timeouts get re-enabled
    static bool last_write_timed_out = false;

    // Setting up a timeout to avoid crashing
    absolute_time_t timeout = make_timeout_time_ms(t->timeout);

    while (!canbus_msg_write_available()) {
        if (time_reached(timeout) || last_write_timed_out || !canbus_initialized) {
            last_write_timed_out = true;
            t->err_code = 1;
            return 0;
        }
        tight_loop_contents();
    }

    last_write_timed_out = false;
    size_t tx_len = canbus_msg_write(buf, len);

    // As we're in packet-oriented mode, we should set errcode if not all the bytes are sent
    if (len != tx_len) {
        t->err_code = 2;
    }

    return tx_len;
}

size_t transport_can_read(uint8_t *buf, size_t len, void *ctx) {
    // absolute_time_t timeout_time = make_timeout_time_ms(t->timeout);
    //  while (!canbus_msg_read_available()) {
    //      if (time_reached(timeout_time) || !canbus_initialized) {
    //          t->err_code = 1;
    //          return 0;
    //      }
    //  }
    if (!canbus_msg_read_available()) {
        return 0;
    }
    return canbus_msg_read(buf, len);

    size_t rx_len = canbus_msg_read(buf, len);

    // As we're in packet-oriented mode, we must set errcode if not all the bytes are received
    if (len != rx_len) {
        t->err_code = 2;
    }

    return rx_len;
}

bool transport_can_init(void *ctx) {
    t = (can_transport_data_t *) ctx;

    if (!canbus_init(t->client_id)) {
        return false;
    }

    return true;
}
