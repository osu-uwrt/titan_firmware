#include <assert.h>
#include <stdbool.h>
#include <string.h>
#include "hardware/irq.h"
#include "pico/multicore.h"

// Public headers
#include "can_pio/canbus.h"

// Private headers
#include "can_bridge.h"

// Done to keep separation between the can_multicore.h header and the internal headers on the other core
// It's not the best [static allocations don't work with this unfortunately], but it works
const size_t canbus_msg_max_length = CANMORE_MAX_MSG_LENGTH;
const size_t canbus_utility_frame_max_length = CANMORE_FRAME_SIZE;

// ========================================
// SIO IRQ Handling Functions
// ========================================

// Callback handling
static canbus_error_cb_t canbus_receive_error_cb = NULL;
static canbus_error_cb_t canbus_internal_error_cb = NULL;
static canbus_cb_t canbus_utility_frame_recv_cb = NULL;
static canbus_cb_t canbus_message_recv_cb = NULL;

void canbus_set_receive_error_cb(canbus_error_cb_t callback) {
    canbus_receive_error_cb = callback;
}

void canbus_set_internal_error_cb(canbus_error_cb_t callback) {
    canbus_internal_error_cb = callback;
}

void canbus_set_utility_frame_recv_cb(canbus_cb_t callback) {
    canbus_utility_frame_recv_cb = callback;
}

void canbus_set_message_recv_cb(canbus_cb_t callback) {
    canbus_message_recv_cb = callback;
}

static void canbus_call_receive_error_cb(int data) {
    if (canbus_receive_error_cb) {
        canbus_receive_error_cb(data);
    }
}

static void canbus_call_internal_error_cb(int data) {
    if (canbus_internal_error_cb) {
        canbus_internal_error_cb(data);
    }
}

static void canbus_call_utility_frame_recv_cb(void) {
    if (canbus_utility_frame_recv_cb) {
        canbus_utility_frame_recv_cb();
    }
}

static void canbus_call_message_recv_cb(void) {
    if (canbus_message_recv_cb) {
        canbus_message_recv_cb();
    }
}


/**
 * @brief Time when the last alive flag received is considered stale, and the core is considered dead.
 * Only valid when canbus_initialized is true
 */
static absolute_time_t canbus_core_alive_timeout;


static void core0_sio_irq() {
    // Process all flags to be received
    while (multicore_fifo_rvalid()) {
        uint32_t recv_flag = multicore_fifo_pop_blocking();

        if (HAS_ERROR_INTERNAL_FLAG(recv_flag)) {
            canbus_call_internal_error_cb(recv_flag & 0x0FFFFFFF);
        } else if (HAS_ERROR_RECEIVE_FLAG(recv_flag)) {
            canbus_call_receive_error_cb(recv_flag & 0x0FFFFFFF);
        } else if (recv_flag == SIO_RECV_UTILITY_FRAME_FLAG) {
            canbus_call_utility_frame_recv_cb();
        } else if (recv_flag == SIO_RECV_MESSAGE_FLAG) {
            canbus_call_message_recv_cb();
        } else if (recv_flag == SIO_CORE_ALIVE_FLAG) {
            canbus_core_alive_timeout = make_timeout_time_ms(CAN_CORE_ALIVE_TIMEOUT_MS);
        } else {
            // Unexpected flag from second core
            canbus_call_internal_error_cb(-1);
        }
    }

    // Check for overflow/underflow
    if (multicore_fifo_get_status() & (SIO_FIFO_ST_WOF_BITS | SIO_FIFO_ST_ROE_BITS)) {
        canbus_call_internal_error_cb(-2);
        multicore_fifo_clear_irq();
    }
}

bool canbus_core_dead(void) {
    return canbus_initialized && time_reached(canbus_core_alive_timeout);
}

void canbus_set_device_in_error(bool device_in_error_state) {
    canbus_device_in_error_state = device_in_error_state;
}


// ========================================
// Initialization Functions
// ========================================

bool canbus_initialized = false;

void canbus_init(uint bitrate,
                 uint client_id,
                 spi_inst_t* spi_channel,
                 uint8_t cs_pin,
                 uint8_t mosi_pin,
                 uint8_t miso_pin,
                 uint8_t sck_pin,
                 uint32_t spi_clock,
                 uint8_t int_pin) {
    assert(!canbus_initialized);

    can_init_cfg_t init_cfg = {
        .bitrate = bitrate,
        .client_id = client_id,
        .spi_channel = spi_channel,
        .cs_pin = cs_pin,
        .mosi_pin = mosi_pin,
        .miso_pin = miso_pin,
        .sck_pin = sck_pin,
        .spi_clock = spi_clock,
        .int_pin = int_pin,
        .gpio_term = -1
    };

    // Startup core and send initial config
    multicore_launch_core1(core1_entry);
    multicore_fifo_push_blocking((uint32_t) &init_cfg);


    // Wait for startup confirmation
    uint32_t response = multicore_fifo_pop_blocking();
    if (response != SIO_STARTUP_DONE_FLAG)
        panic("Unexpected response from core1 startup: %d", response);

    canbus_core_alive_timeout = make_timeout_time_ms(CAN_CORE_ALIVE_TIMEOUT_MS);


    // Enable IRQ for core 1 heartbeat and fault reporting
    irq_set_exclusive_handler(SIO_IRQ_PROC0, core0_sio_irq);
    irq_set_enabled(SIO_IRQ_PROC0, true);

    canbus_initialized = true;
}

bool canbus_check_online(void) {
    assert(canbus_initialized);

    return !time_reached(canbus_heartbeat_timeout);
}


// ========================================
// Msg Queue Functions
// ========================================

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


// ========================================
// Utility Queue Functions
// ========================================

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