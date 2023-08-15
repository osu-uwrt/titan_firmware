#ifndef UART_TELEMETRY_H
#define UART_TELEMETRY_H

#include <stddef.h>
#include <stdint.h>

enum uart_telem_recv_type { ESC_TELEM_FRAME, ESC_INFO_FRAME };

struct __attribute__((__packed__)) esc_telem_pkt {
    uint8_t temperature;
    uint8_t voltage_high;
    uint8_t voltage_low;
    uint8_t current_high;
    uint8_t current_low;
    uint8_t consumption_high;
    uint8_t consumption_low;
    uint8_t rpm_high;
    uint8_t rpm_low;
    uint8_t crc;
};

struct __attribute__((__packed__)) esc_info_pkt {
    uint8_t esc_serial[12];
    uint8_t response_version;
    uint8_t fw_revision;
    uint8_t fw_subrevision;
    uint8_t _reserved_a;
    uint8_t direction_reversed;
    uint8_t direction_3d_active;
    uint8_t low_voltage_limit;
    uint8_t current_limit;
    uint8_t led0_status;
    uint8_t led1_status;
    uint8_t led2_status;
    uint8_t led3_status;
    uint8_t _reserved_b[7];
    uint8_t esc_signature[31];
    uint8_t crc;
};

struct uart_telem_buffer {
    union {
        uint8_t raw[1];
        struct esc_telem_pkt telem_pkt;
        //        struct esc_info_pkt info_pkt;     // Info not implemented on APD ESCs
    } buffer;
    enum uart_telem_recv_type type;
    uint8_t crc;
    size_t recv_index;
    int thruster;
};

#endif
