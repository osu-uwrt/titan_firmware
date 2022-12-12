#include <assert.h>
#include <stdbool.h>
#include <string.h>
#include "hardware/irq.h"
#include "pico/stdlib.h"
#include "pico/multicore.h"

// Public headers
#include "can_pio/canbus.h"

// Private headers
#include "can_bridge.h"

// Done to keep separation between the can_multicore.h header and the internal headers on the other core
// It's not the best [static allocations don't work with this unfortunately], but it works
const size_t canbus_msg_max_length = CANMORE_MAX_MSG_LENGTH;
const size_t canbus_utility_frame_max_length = CANMORE_FRAME_SIZE;

static void core0_sio_irq() {
    // Just record the latest entry
    //while (multicore_fifo_rvalid())
    //    core0_rx_val = multicore_fifo_pop_blocking();

    //multicore_fifo_clear_irq();
}

// Ensures that functions aren't called without startup up the CAN bus
bool canbus_initialized = false;

void canbus_init(unsigned pio_num, unsigned bitrate, unsigned client_id, unsigned gpio_rx, unsigned gpio_tx, int gpio_term) {
    assert(!canbus_initialized);

    can_init_cfg_t init_cfg = {
        .pio_num = pio_num,
        .bitrate = bitrate,
        .client_id = client_id,
        .gpio_rx = gpio_rx,
        .gpio_tx = gpio_tx,
        .gpio_term = gpio_term
    };

    // Startup core and send initial config
    multicore_launch_core1(core1_entry);
    multicore_fifo_push_blocking((uint32_t) &init_cfg);

    // Wait for startup confirmation
    uint32_t response = multicore_fifo_pop_blocking();
    if (response != CORE1_STARTUP_DONE_FLAG)
        panic("Unexpected response from core1 startup: %d", response);

    // TODO: Fixup IRQs
    //irq_set_exclusive_handler(SIO_IRQ_PROC0, core0_sio_irq);
    //irq_set_enabled(SIO_IRQ_PROC0, true);

    canbus_initialized = true;
}

bool canbus_check_online(void) {
    assert(canbus_initialized);

    return !time_reached(canbus_heartbeat_timeout);
}

bool canbus_msg_read_available(void) {
    assert(canbus_initialized);

    return !QUEUE_EMPTY(&msg_receive_queue);
}

size_t canbus_msg_read(uint8_t *buf, size_t len) {
    assert(canbus_initialized);

    if (QUEUE_EMPTY(&msg_receive_queue)) {
        return 0;
    }

    struct canmore_msg *entry = QUEUE_CUR_READ_ENTRY(&msg_receive_queue);

    size_t copy_len = entry->length;
    if (copy_len > len) {
        copy_len = len;
    }

    memcpy(buf, entry->data, copy_len);
    QUEUE_MARK_READ_DONE(&msg_receive_queue);

    return copy_len;
}

bool canbus_msg_write_available(void) {
    assert(canbus_initialized);

    return !QUEUE_FULL(&msg_transmit_queue);
}

size_t canbus_msg_write(const uint8_t *buf, size_t len) {
    assert(canbus_initialized);

    if (QUEUE_FULL(&msg_transmit_queue)) {
        return 0;
    }

    // Length bounds checking
    if (len > CANMORE_MAX_MSG_LENGTH) {
        len = CANMORE_MAX_MSG_LENGTH;
    }
    if (len == 0) {
        return 0;
    }

    struct canmore_msg *entry = QUEUE_CUR_WRITE_ENTRY(&msg_transmit_queue);
    memcpy(entry->data, buf, len);
    entry->length = len;
    QUEUE_MARK_WRITE_DONE(&msg_transmit_queue);

    return len;
}

bool canbus_utility_frame_read_available(void) {
    assert(canbus_initialized);

    return !QUEUE_EMPTY(&utility_receive_queue);
}

size_t canbus_utility_frame_read(uint32_t *channel_out, uint8_t *buf, size_t len) {
    assert(canbus_initialized);

    if (QUEUE_EMPTY(&utility_receive_queue)) {
        return 0;
    }

    // Copy out data from buffer from second core
    struct canmore_utility_frame *frame = QUEUE_CUR_READ_ENTRY(&utility_receive_queue);

    size_t copy_len = frame->dlc;
    if (copy_len > len)
        copy_len = len;

    memcpy(buf, frame->data, copy_len);
    *channel_out = frame->channel;
    QUEUE_MARK_READ_DONE(&utility_receive_queue);

    return copy_len;
}

bool canbus_utility_frame_write_available(void) {
    assert(canbus_initialized);

    return !QUEUE_FULL(&utility_transmit_queue);
}

size_t canbus_utility_frame_write(uint32_t channel, uint8_t *buf, size_t len) {
    assert(canbus_initialized);

    if (QUEUE_FULL(&utility_transmit_queue)) {
        return 0;
    }

    // Check channel ID
    if (channel > (1<<CANMORE_NOC_LENGTH)) {
        return 0;
    }

    // Length bounds checking
    if (len > CANMORE_FRAME_SIZE) {
        len = CANMORE_FRAME_SIZE;
    }
    if (len == 0) {
        return 0;
    }

    // Copy in frame to buffer for second core to fetch
    struct canmore_utility_frame *frame = QUEUE_CUR_WRITE_ENTRY(&utility_transmit_queue);
    memcpy(frame->data, buf, len);
    frame->dlc = len;
    frame->channel = channel;
    QUEUE_MARK_WRITE_DONE(&utility_transmit_queue);

    return len;
}