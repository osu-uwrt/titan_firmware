/**
 * Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#if !defined(LIB_TINYUSB_HOST) && !defined(LIB_TINYUSB_DEVICE)
#include "tusb.h"

#include "pico/time.h"
#include "pico/stdio/driver.h"
#include "pico/binary_info.h"
#include "pico/mutex.h"
#include "hardware/irq.h"
#include "hardware/gpio.h"

#define USBD_ITF_STDIO_CDC       (0)
#define USBD_ITF_SECONDARY_CDC   (1)

static_assert(PICO_STDIO_USB_LOW_PRIORITY_IRQ > RTC_IRQ, ""); // note RTC_IRQ is currently the last one
static mutex_t stdio_usb_mutex;
static mutex_t unsent_buffer_mutex;

static const int unsent_buffer_size = PICO_STDIO_USB_UNSENT_BUFFER_SIZE;
static char unsent_buffer[PICO_STDIO_USB_UNSENT_BUFFER_SIZE];
static int unsent_buffer_next_out = 0;
static int unsent_buffer_next_in = 0;
static bool unsent_buffer_overflow = false;
static bool buffer_mutex_data_drop = false;
static const char overflow_msg[] = "---DATA DROPPED---\r\n";
static const char mutex_msg[] = "---DATA LOST MUTEX---\r\n";

void _write_unsent_buffer(const char* buf, int length);

static int _stdio_usb_out_data(const char *buf, int length) {
    bool sent_all = true;
    int i;
    for (i = 0; i < length;) {
        int n = length - i;
        int avail = (int) tud_cdc_n_write_available(USBD_ITF_STDIO_CDC);
        if (n > avail) n = avail;
        if (n) {
            int n2 = (int) tud_cdc_n_write(USBD_ITF_STDIO_CDC, buf + i, (uint32_t)n);
            tud_task();
            tud_cdc_n_write_flush(USBD_ITF_STDIO_CDC);
            i += n2;
        } else {
            break;
        }
    }
    return i;
}

static inline int positive_modulo(int i, int n) {
    return (i % n + n) % n;
}

bool _try_handle_unsent_buffer(void) {
    if (!mutex_try_enter(&unsent_buffer_mutex, NULL)) {
        return false;
    }

    if (unsent_buffer_overflow) {
        int sent = _stdio_usb_out_data(overflow_msg, sizeof(overflow_msg));
        if (sent == sizeof(overflow_msg)) {
            unsent_buffer_overflow = false;
        } else {
            return false;
        }
    }

    if (buffer_mutex_data_drop) {
        int sent = _stdio_usb_out_data(mutex_msg, sizeof(mutex_msg));
        if (sent == sizeof(mutex_msg)) {
            buffer_mutex_data_drop = false;
        } else {
            return false;
        }
    }

    int sent;
    if (unsent_buffer_next_in >= unsent_buffer_next_out) {
        sent = _stdio_usb_out_data(&unsent_buffer[unsent_buffer_next_out], unsent_buffer_next_in - unsent_buffer_next_out);
    } else {
        sent = _stdio_usb_out_data(&unsent_buffer[unsent_buffer_next_out], unsent_buffer_size - unsent_buffer_next_out);
        if (sent == unsent_buffer_size - unsent_buffer_next_out)
            sent += _stdio_usb_out_data(unsent_buffer, unsent_buffer_next_in);
    }
    unsent_buffer_next_out = ((unsent_buffer_next_out + sent) % unsent_buffer_size);

    mutex_exit(&unsent_buffer_mutex);
    return unsent_buffer_next_out == unsent_buffer_next_in;
}

void _write_unsent_buffer(const char* buf, int length) {
    uint32_t owner;
    if (!mutex_try_enter(&unsent_buffer_mutex, &owner)) {
        if (owner == get_core_num()) {  // would deadlock otherwise
            buffer_mutex_data_drop = true;
            return;
        }
        mutex_enter_blocking(&unsent_buffer_mutex);
    }
    
    // If the data is too big for buffer, truncate to end
    if (length >= unsent_buffer_size) {
        buf = &buf[length - (unsent_buffer_size - 1)];
        length = unsent_buffer_size - 1;
        unsent_buffer_overflow = true;
    }

    // Check if buffer is going to overflow, and fix unsent_buffer_next_out if it will
    if (length >= unsent_buffer_size - positive_modulo(unsent_buffer_next_in - unsent_buffer_next_out, unsent_buffer_size)) {
        unsent_buffer_overflow = true;
        unsent_buffer_next_out = ((unsent_buffer_next_in + length) % unsent_buffer_size) + 1;
    }

    // Handle cases for wrapping ring buffer
    if (length >= (unsent_buffer_size - unsent_buffer_next_in)) {
        memcpy(&unsent_buffer[unsent_buffer_next_in], buf, unsent_buffer_size - unsent_buffer_next_in);
        buf = &buf[unsent_buffer_size - unsent_buffer_next_in];
        length -= (unsent_buffer_size - unsent_buffer_next_in);
        unsent_buffer_next_in = 0;
    }

    // Handle general case (so long as there's data to be processed)
    if (length > 0) {
        memcpy(&unsent_buffer[unsent_buffer_next_in], buf, length);
        unsent_buffer_next_in += length;
    }

    mutex_exit(&unsent_buffer_mutex);
}

static void low_priority_worker_irq(void) {
    // if the mutex is already owned, then we are in user code
    // in this file which will do a tud_task itself, so we'll just do nothing
    // until the next tick; we won't starve
    if (mutex_try_enter(&stdio_usb_mutex, NULL)) {
        tud_task();
        if (tud_cdc_n_connected(USBD_ITF_STDIO_CDC)) {
            _try_handle_unsent_buffer();
        }
        mutex_exit(&stdio_usb_mutex);
    }
}

static int64_t timer_task(__unused alarm_id_t id, __unused void *user_data) {
    irq_set_pending(PICO_STDIO_USB_LOW_PRIORITY_IRQ);
    return PICO_STDIO_USB_TASK_INTERVAL_US;
}

static void stdio_usb_out_chars(const char *buf, int length) {
    uint32_t owner;
    if (!mutex_try_enter(&stdio_usb_mutex, &owner)) {
        if (owner == get_core_num()) { 
            // As a last ditch attempt try to send to unsent buffer
            // If this is also locked it will fail, but should allow writing when usb is locked
            // This becomes useful if usb is commonly locked due to communication on the secondary port
            _write_unsent_buffer(buf, length);
            return; // would deadlock otherwise
        }
        mutex_enter_blocking(&stdio_usb_mutex);
    }
    if (tud_cdc_n_connected(USBD_ITF_STDIO_CDC) && _try_handle_unsent_buffer()) {
        int sent = _stdio_usb_out_data(buf, length);
        
        // If not all data was sent, put remaining in the buffer to be handled later
        if (sent < length) {
            _write_unsent_buffer(buf + sent, length - sent);
        }
    } else {
        // If not connected then buffer data
        _write_unsent_buffer(buf, length);

    }
    mutex_exit(&stdio_usb_mutex);
}

int stdio_usb_in_chars(char *buf, int length) {
    uint32_t owner;
    if (!mutex_try_enter(&stdio_usb_mutex, &owner)) {
        if (owner == get_core_num()) return PICO_ERROR_NO_DATA; // would deadlock otherwise
        mutex_enter_blocking(&stdio_usb_mutex);
    }
    int rc = PICO_ERROR_NO_DATA;
    if (tud_cdc_n_connected(USBD_ITF_STDIO_CDC) && tud_cdc_n_available(USBD_ITF_STDIO_CDC)) {
        tud_task();
        int count = (int) tud_cdc_n_read(USBD_ITF_STDIO_CDC, buf, (uint32_t) length);
        rc =  count ? count : PICO_ERROR_NO_DATA;
    }
    mutex_exit(&stdio_usb_mutex);
    return rc;
}

stdio_driver_t stdio_usb = {
    .out_chars = stdio_usb_out_chars,
    .in_chars = stdio_usb_in_chars,
#if PICO_STDIO_ENABLE_CRLF_SUPPORT
    .crlf_enabled = PICO_STDIO_USB_DEFAULT_CRLF
#endif
};

bool dual_usb_init(void) {
#if !PICO_NO_BI_STDIO_USB
    bi_decl_if_func_used(bi_program_feature("Dual USB stdin / stdout"));
#endif

    // initialize TinyUSB
    tusb_init();

    irq_set_exclusive_handler(PICO_STDIO_USB_LOW_PRIORITY_IRQ, low_priority_worker_irq);
    irq_set_enabled(PICO_STDIO_USB_LOW_PRIORITY_IRQ, true);

    mutex_init(&stdio_usb_mutex);
    mutex_init(&unsent_buffer_mutex);
    bool rc = add_alarm_in_us(PICO_STDIO_USB_TASK_INTERVAL_US, timer_task, NULL, true);
    if (rc) {
        stdio_set_driver_enabled(&stdio_usb, true);
#if PICO_STDIO_USB_CONNECT_WAIT_TIMEOUT_MS
#if PICO_STDIO_USB_CONNECT_WAIT_TIMEOUT_MS > 0
        absolute_time_t until = make_timeout_time_ms(PICO_STDIO_USB_CONNECT_WAIT_TIMEOUT_MS);
#else
        absolute_time_t until = at_the_end_of_time;
#endif
        do {
            if (stdio_usb_connected()) {
#if PICO_STDIO_USB_POST_CONNECT_WAIT_DELAY_MS != 0
                sleep_ms(PICO_STDIO_USB_POST_CONNECT_WAIT_DELAY_MS);
#endif
                break;
            }
            sleep_ms(10);
        } while (!time_reached(until));
#endif
    }
    return rc;
}

bool stdio_usb_connected(void) {
    return tud_cdc_n_connected(USBD_ITF_STDIO_CDC);
}

/*
 * Secondary USB Interface
 */

int secondary_usb_out_chars(const char *buf, int length) {
    static uint64_t last_avail_time_secondary;
    uint32_t owner;
    int chars_sent = 0;
    if (!mutex_try_enter(&stdio_usb_mutex, &owner)) {
        if (owner == get_core_num()) return 0; // would deadlock otherwise
        mutex_enter_blocking(&stdio_usb_mutex);
    }
    if (tud_cdc_n_connected(USBD_ITF_SECONDARY_CDC)) {
        for (int i = 0; i < length;) {
            int n = length - i;
            int avail = (int) tud_cdc_n_write_available(USBD_ITF_SECONDARY_CDC);
            if (n > avail) n = avail;
            if (n) {
                int n2 = (int) tud_cdc_n_write(USBD_ITF_SECONDARY_CDC, buf + i, (uint32_t)n);
                tud_task();
                tud_cdc_n_write_flush(USBD_ITF_SECONDARY_CDC);
                i += n2;
                chars_sent += n2;
                last_avail_time_secondary = time_us_64();
            } else {
                tud_task();
                tud_cdc_n_write_flush(USBD_ITF_SECONDARY_CDC);
                if (!tud_cdc_n_connected(USBD_ITF_SECONDARY_CDC) ||
                    (!tud_cdc_n_write_available(USBD_ITF_SECONDARY_CDC) && time_us_64() > last_avail_time_secondary + PICO_STDIO_USB_STDOUT_TIMEOUT_US)) {
                    break;
                }
            }
        }
    } else {
        // reset our timeout
        last_avail_time_secondary = 0;
    }
    mutex_exit(&stdio_usb_mutex);
    return chars_sent;
}

int secondary_usb_in_chars(char *buf, int length) {
    uint32_t owner;
    if (!mutex_try_enter(&stdio_usb_mutex, &owner)) {
        if (owner == get_core_num()) return PICO_ERROR_NO_DATA; // would deadlock otherwise
        mutex_enter_blocking(&stdio_usb_mutex);
    }
    int rc = PICO_ERROR_NO_DATA;
    if (tud_cdc_n_connected(USBD_ITF_SECONDARY_CDC) && tud_cdc_n_available(USBD_ITF_SECONDARY_CDC)) {
        tud_task();
        int count = (int) tud_cdc_n_read(USBD_ITF_SECONDARY_CDC, buf, (uint32_t) length);
        rc =  count ? count : PICO_ERROR_NO_DATA;
    }
    mutex_exit(&stdio_usb_mutex);
    return rc;
}

bool secondary_usb_connected(void) {
    return tud_cdc_n_connected(USBD_ITF_SECONDARY_CDC);
}

#else
#include "pico/stdio_usb.h"
#warning stdio USB was configured, but is being disabled as TinyUSB is explicitly linked
bool stdio_usb_init(void) {
    return false;
}
#endif
