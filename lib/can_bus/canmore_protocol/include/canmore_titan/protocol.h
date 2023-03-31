#ifndef CANMORE_TITAN__PROTOCOL_H
#define CANMORE_TITAN__PROTOCOL_H

#include "canmore/protocol.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
 * CANmore Titan-Specific Modifications
 * ************************************
 *
 * This file contains titan firmware specific modifications to the CANmore protocol.
 * This is primarily to define any application-specific fields of the CANmore protocol.
 *
 * Refer to each of the children header files for information on each
 *
 * Utility Channel Allocations
 * ===========================
 * The utility channels defined by the CANmore protocol are allocated as follows:
 *
 * 0 - 13: RFU
 *
 * 14: Control Interface
 *   If Client Heartbeat:Mode = Boot Delay:
 *      Follows Bootloader Request
 *   If Client Heartbeat:Mode = Normal Mode
 *      Follows Client Debug Protocol
 *   If Client Heartbeat:Mode = Bootloader Mode
 *      Follows Bootloader Protocol
 *
 * 15: Heartbeat (Defined by CANmore)
 *
 *
 * Heartbeat Extra Data
 * ====================
 *
 * The heartbeat extra data field is populated with the following data:
 *   +-*-+-*-+-*-*-*-+
 *   | T | V |   M   |
 *   +-*-+-*-+-*-*-*-+
 *     7   6   5   3
 *
 * M (Mode): The current control interface mode (for reporting an implemented control interface)
 *    - 000: No Control Interface Present
 *    - 001: Normal Mode
 *    - 010: Bootloader Mode
 *    - 011-100: RFU
 *    - 101: Boot Delay - Sent on boot to notify the client is online and allow entering bootloader mode
 *    - 110-111: RFU
 * V (Term Valid): 1 if the termination state is valid, 0 if it should be ignored
 * T (Term Enabled): 1 if the node has its termination resistor enabled, 0 if disabled
 */

#define CANMORE_TITAN_CHAN_CONTROL_INTERFACE 14
#define CANMORE_TITAN_CONTROL_INTERFACE_BOOTLOADER_REQUEST {0xB0, 0x07, 0x10, 0xAD}

// Control Interface Mode Values (for both Heartbeat and Control Interface)
#define CANMORE_TITAN_CONTROL_INTERFACE_MODE_NONE        0b000u
#define CANMORE_TITAN_CONTROL_INTERFACE_MODE_NORMAL      0b001u
#define CANMORE_TITAN_CONTROL_INTERFACE_MODE_BOOTLOADER  0b010u
#define CANMORE_TITAN_CONTROL_INTERFACE_MODE_BOOT_DELAY  0b101u

// Heartbeat field lengths
#define CANMORE_TITAN_HEARTBEAT_MODE_LENGTH          3
#define CANMORE_TITAN_HEARTBEAT_TERM_VALID_LENGTH    1
#define CANMORE_TITAN_HEARTBEAT_TERM_ENABLED_LENGTH  1

// Heartbeat offsets
#define CANMORE_TITAN_HEARTBEAT_MODE_OFFSET          CANMORE_HEARTBEAT_EXTRA_OFFSET
#define CANMORE_TITAN_HEARTBEAT_TERM_VALID_OFFSET    (CANMORE_TITAN_HEARTBEAT_MODE_OFFSET + CANMORE_TITAN_HEARTBEAT_MODE_LENGTH)
#define CANMORE_TITAN_HEARTBEAT_TERM_ENABLED_OFFSET  (CANMORE_TITAN_HEARTBEAT_TERM_VALID_OFFSET + CANMORE_TITAN_HEARTBEAT_TERM_VALID_LENGTH)

typedef union __attribute__((__packed__)) canmore_titan_heartbeat {
    uint8_t data;
    struct canmore_titan_heartbeat_packet {
        uint8_t cnt:CANMORE_HEARTBEAT_CNT_LENGTH;
        uint8_t error:CANMORE_HEARTBEAT_ERROR_LENGTH;
        uint8_t mode:CANMORE_TITAN_HEARTBEAT_MODE_LENGTH;
        uint8_t term_valid:CANMORE_TITAN_HEARTBEAT_TERM_VALID_LENGTH;
        uint8_t term_enabled:CANMORE_TITAN_HEARTBEAT_TERM_ENABLED_LENGTH;
    } pkt;
} canmore_titan_heartbeat_t;

#define CANMORE_CALC_TITAN_HEARTBEAT_DATA(cnt, error, mode, term_valid, term_enabled) \
    ( \
        (((cnt) & ((1u<<CANMORE_TITAN_HEARTBEAT_CNT_LENGTH) - 1u)) << CANMORE_TITAN_HEARTBEAT_CNT_OFFSET) | \
        (((error) & ((1u<<CANMORE_TITAN_HEARTBEAT_ERROR_LENGTH) - 1u)) << CANMORE_TITAN_HEARTBEAT_ERROR_OFFSET) | \
        (((mode) & ((1u<<CANMORE_TITAN_HEARTBEAT_MODE_LENGTH) - 1u)) << CANMORE_TITAN_HEARTBEAT_MODE_OFFSET) | \
        (((term_valid) & ((1u<<CANMORE_TITAN_HEARTBEAT_TERM_VALID_LENGTH) - 1u)) << CANMORE_TITAN_HEARTBEAT_TERM_VALID_OFFSET) | \
        (((term_enabled) & ((1u<<CANMORE_TITAN_HEARTBEAT_TERM_ENABLED_LENGTH) - 1u)) << CANMORE_TITAN_HEARTBEAT_TERM_ENABLED_OFFSET) \
    )

#ifdef __cplusplus
}
#endif

#endif