#ifndef CANMORE__PROTOCOL_H
#define CANMORE__PROTOCOL_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * CANmore Specification
 * **********************
 * CANmore is a transport layer definition, addressing scheme, and protocol to allow bandwidth-efficient communication
 * on ISO 11898-1 CAN networks carrying DDS-XRCE messages.
 *
 *
 * Purpose
 * =======
 * This speicfication is designed to allow DDS-XRCE communication with minimal framing overhead. Presently, XRCE-DDS has
 * two transport typesL stream and packet. Packet transports send raw DDS-XRCE messages over the transport, but does not
 * allow fragmentation. In packet transport mode, if the MTU is less than the message size, the message will be dropped.
 * Due to CAN's 8-byte MTU, DDS-XRCE messages are unable to fit inside a single CAN frame. Stream transport is designed
 * to handle fragmentation, but adds 8 bytes of overhead for each DDS message. Due to the low-bandwidth available on CAN
 * bus, special attention must be paid to unnessary traffic on the bus to allow for maximum useful bandwidth on the bus.
 * Stream transport, additionally, has several fields unnecessary for CAN bus, such as synchronization, device ID, and
 * error correction fields, all of which are already present in the CAN specification.
 *
 * This specification allows using packet transport with minimal overhead by using the message ID field to contain the
 * required data to reassemble a fragmented packet, keeping the data fields for the DDS-XRCE message. Additionally, bits
 * are used to allow out-of-band traffic to be sent in parallel over the bus, such as diagnostics or firmware updates.
 *
 * TODO: Finish writing this specification, definitions
 *
 *
 * Definitions
 * ===========
 * Agent: The CAN node with the DDS-XRCE agent
 * Client: A CAN node running a DDS-XRCE client
 * Client ID:
 * Channel:
 * Frame: An ISO 11898-1 CAN Bus Frame
 * Node: An ISO 11898-1 CAN compliant endpoint
 * Message: A single DDS-XRCE packet. This can be spread over several CAN frames
 * Message (Type) Frame: A CANmore frame type designed to carry fragmented messages
 * NOC: Number or Channel, This field represents the message sequence number or utility frame channel, depending on frame type
 * RFU: Reserved for Future Use. This field does not curerntly have a defined value, but could be defined in the future. Should be set to 0
 * Sequence Number:
 *
 *
 * Bus Hierarchy
 * =============
 * This bus has a single central agent node with one or several other client nodes. Each client is assigned a unique ID.
 * All message type communication on the network is between a client and the controller. Two clients cannot communicate
 * directly.
 *
 *                        +-------+
 *                      /-| ID 10 |
 *                    /-  +-------+
 *                  /-
 * +------------+ /-      +-------+
 * | Controller |---------| ID 11 |
 * +------------+ \-      +-------+
 *                  \-
 *                    \-  +-------+
 *                      \-| ID 12 |
 *                        +-------+
 *
 *
 * Frame Types
 * ===========
 * Two frame types are supported on the CAN bus: message and utility frames.
 *
 * Message frames carry a fragmented message, such as an DDS-XRCE message. The packet is split up into individual
 * message frames, sent over the bus, and re-assmbled to form a single packet. If any frames are dropped or during this
 * transfer, the entire message is dropped.
 *
 * Utility frames carry out-of-band data, such as diagnostics, heartbeats, and firmware updates. Every utility frame
 * has a channel, 0-15, which allows for several low-bandwidth out-of-band queues for client applications.
 *
 *
 * Frame Format
 * ============
 * All of the protocol and framing information is packed inside the identifier field of the CAN frame, allowing for all
 * data fields of the frame to be used by higher-level protocols, minimizing overhead. CANmore will use the standard CAN
 * frame for all utility type frames. For a message type, every frame is standard except the last, which is an extended
 * frame, where the additional 18 bits are allocated for a CRC.
 *
 * RTR frames are not supported, nor are frames with the DLC field set to a value greater than 8.
 *
 *
 * Standard CAN Frame Identifier Format:
 *   +-*-*-*-*-*-+-*-+-*-+-*-*-*-*-+
 *   | CLIENT ID | T | D |   NOC   |  (Standard 11-bit ID)
 *   +-*-*-*-*-*-+-*-+-*-+-*-*-*-*-+
 *    10       6   5   4   3     0
 *
 * Extended CAN Frame Identifier Format:
 *   +-*-*-*-*-*-+-*-+-*-+-*-*-*-*-+-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-+
 *   | CLIENT ID | T | D |   NOC   |                 CRC                 |  (Extended 29-bit ID)
 *   +-*-*-*-*-*-+-*-+-*-+-*-*-*-*-+-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-+
 *    28      24  23  22  21    18  17                                 0
 *
 * CLIENT ID: The client this frame is intended for (if sent from the agent) or originating from (if sent from the client).
 *     Up to 32 clients supported, due to CAN arbitration, lower client IDs have higher priority
 * TYPE (T): Signifies the frame type. 0 for message type frames; 1 for utility type frames.
 * DIRECTION (D): The direction of the request. 0 for client to agent, 1 for agent to client
 * NOC: [Number or Channel] The sequence number (if message type) or communication channel (if utility type).
 * CRC: A CRC-18 checksum to ensure the message is re-assembled properly. Only valid in message type frames.
 *
 *
 * Message Frame Protocol
 * ======================
 * Frames with the type set to
 *
 * TODO:
 * No zero length packets
 * CRC 18 definition: Polynomial: 0x23979, Initial Value: 0x3FFFF
 *
 *
 * Utility Frame Protocol
 * ======================
 *
 * Heartbeat Channel
 * -----------------
 * Channel 15 (all 1s) is reserved for heartbeat transmission from the client. This channel does not have any agent-to-
 * client requests. The interval for this heartbeat is up to the implementer.
 *
 * Heartbeat frames have a DLC of 1, with the following format:
 *   +-*-*-*-*-*-+-*-+-*-*-+
 *   |   EXTRA   | E | CNT |
 *   +-*-*-*-*-*-+-*-+-*-*-+
 *     7       3   2   1 0
 *
 * CNT: An increasing mod-4 counter incremented for each packet sent. This can be use to detect dropped packets
 * E (ERROR): A single bit representing the status of the client (0 for normal, 1 for error-state)
 * EXTRA: 5 bits of extra data available to the implementer
*/


// ========================================
// CANmore Specification Constants
// ========================================

// Length of CAN frame
#define CANMORE_FRAME_SIZE 8

// Identifier Field Lengths
#define CANMORE_CRC_LENGTH            18
#define CANMORE_NOC_LENGTH            4
#define CANMORE_DIRECTION_LENGTH      1
#define CANMORE_TYPE_LENGTH           1
#define CANMORE_CLIENT_ID_LENGTH      5

// Standard Identifier Offsets
#define CANMORE_STD_NOC_OFFSET        0
#define CANMORE_STD_DIRECTION_OFFSET  (CANMORE_STD_NOC_OFFSET + CANMORE_NOC_LENGTH)
#define CANMORE_STD_TYPE_OFFSET       (CANMORE_STD_DIRECTION_OFFSET + CANMORE_DIRECTION_LENGTH)
#define CANMORE_STD_CLIENT_ID_OFFSET  (CANMORE_STD_TYPE_OFFSET + CANMORE_TYPE_LENGTH)

// Extended Identifier Offsets
#define CANMORE_CRC_CRC_OFFSET        0
#define CANMORE_CRC_NOC_OFFSET        (CANMORE_CRC_CRC_OFFSET + CANMORE_CRC_LENGTH)
#define CANMORE_CRC_DIRECTION_OFFSET  (CANMORE_CRC_NOC_OFFSET + CANMORE_NOC_LENGTH)
#define CANMORE_CRC_TYPE_OFFSET       (CANMORE_CRC_DIRECTION_OFFSET + CANMORE_DIRECTION_LENGTH)
#define CANMORE_CRC_CLIENT_ID_OFFSET  (CANMORE_CRC_TYPE_OFFSET + CANMORE_TYPE_LENGTH)

// Direction types
#define CANMORE_DIRECTION_CLIENT_TO_AGENT  0
#define CANMORE_DIRECTION_AGENT_TO_CLIENT  1

// Frame Types
#define CANMORE_TYPE_MSG  0
#define CANMORE_TYPE_UTIL 1


// Heartbeat field lengths
#define CANMORE_CHAN_HEARTBEAT          15
#define CANMORE_HEARTBEAT_CNT_LENGTH    2
#define CANMORE_HEARTBEAT_ERROR_LENGTH  1
#define CANMORE_HEARTBEAT_EXTRA_LENGTH  5

// Heartbeat offsets
#define CANMORE_HEARTBEAT_CNT_OFFSET    0
#define CANMORE_HEARTBEAT_ERROR_OFFSET  (CANMORE_HEARTBEAT_CNT_OFFSET + CANMORE_HEARTBEAT_CNT_LENGTH)
#define CANMORE_HEARTBEAT_EXTRA_OFFSET  (CANMORE_HEARTBEAT_ERROR_OFFSET + CANMORE_HEARTBEAT_ERROR_LENGTH)

// ========================================
// Decoding Unions
// ========================================

typedef union __attribute__((__packed__)) canmore_id {
    uint32_t identifier;
    struct canmore_id_std {
        uint32_t noc:CANMORE_NOC_LENGTH;
        uint32_t direction:CANMORE_DIRECTION_LENGTH;
        uint32_t type:CANMORE_TYPE_LENGTH;
        uint32_t client_id:CANMORE_CLIENT_ID_LENGTH;
    } pkt_std;
    struct canmore_id_ext {
        uint32_t crc:CANMORE_CRC_LENGTH;
        uint32_t noc:CANMORE_NOC_LENGTH;
        uint32_t direction:CANMORE_DIRECTION_LENGTH;
        uint32_t type:CANMORE_TYPE_LENGTH;
        uint32_t client_id:CANMORE_CLIENT_ID_LENGTH;
    } pkt_ext;
} canmore_id_t;

typedef union __attribute__((__packed__)) canmore_heartbeat {
    uint8_t data;
    struct canmore_heartbeat_packet {
        uint8_t cnt:CANMORE_HEARTBEAT_CNT_LENGTH;
        uint8_t error:CANMORE_HEARTBEAT_ERROR_LENGTH;
        uint8_t extra:CANMORE_HEARTBEAT_EXTRA_LENGTH;
    } pkt;
} canmore_heartbeat_t;


// ========================================
// Calculation Macros
// ========================================

#define CANMORE_CALC_STD_ID(client_id, type, direction, noc) \
    ( \
        (((client_id) & ((1u<<CANMORE_CLIENT_ID_LENGTH) - 1)) << CANMORE_STD_CLIENT_ID_OFFSET) | \
        (((type) & ((1u<<CANMORE_TYPE_LENGTH) - 1u)) << CANMORE_STD_TYPE_OFFSET) | \
        (((direction) & ((1u<<CANMORE_DIRECTION_LENGTH) - 1u)) << CANMORE_STD_DIRECTION_OFFSET) | \
        (((noc) & ((1u<<CANMORE_NOC_LENGTH) - 1u)) << CANMORE_STD_NOC_OFFSET) \
    )

#define CANMORE_CALC_EXT_ID(client_id, type, direction, noc, crc) \
    ( \
        (((client_id) & ((1u<<CANMORE_CLIENT_ID_LENGTH) - 1u)) << CANMORE_CRC_CLIENT_ID_OFFSET) | \
        (((type) & ((1u<<CANMORE_TYPE_LENGTH) - 1u)) << CANMORE_CRC_TYPE_OFFSET) | \
        (((direction) & ((1u<<CANMORE_DIRECTION_LENGTH) - 1u)) << CANMORE_CRC_DIRECTION_OFFSET) | \
        (((noc) & ((1u<<CANMORE_NOC_LENGTH) - 1u)) << CANMORE_CRC_NOC_OFFSET) | \
        (((crc) & ((1u<<CANMORE_CRC_LENGTH) - 1u)) << CANMORE_CRC_CRC_OFFSET) \
    )

// Message standard ID
#define CANMORE_CALC_MSG_ID(client_id, direction, seq_num) CANMORE_CALC_STD_ID(client_id, CANMORE_TYPE_MSG, direction, seq_num)
#define CANMORE_CALC_MSG_ID_A2C(client_id, seq_num) CANMORE_CALC_MSG_ID(client_id, CANMORE_DIRECTION_AGENT_TO_CLIENT, seq_num)
#define CANMORE_CALC_MSG_ID_C2A(client_id, seq_num) CANMORE_CALC_MSG_ID(client_id, CANMORE_DIRECTION_CLIENT_TO_AGENT, seq_num)

// Message extended ID
#define CANMORE_CALC_MSG_EXT_ID(client_id, direction, seq_num, crc) CANMORE_CALC_EXT_ID(client_id, CANMORE_TYPE_MSG, direction, seq_num, crc)
#define CANMORE_CALC_MSG_EXT_ID_A2C(client_id, seq_num, crc) CANMORE_CALC_MSG_EXT_ID(client_id, CANMORE_DIRECTION_AGENT_TO_CLIENT, seq_num, crc)
#define CANMORE_CALC_MSG_EXT_ID_C2A(client_id, seq_num, crc) CANMORE_CALC_MSG_EXT_ID(client_id, CANMORE_DIRECTION_CLIENT_TO_AGENT, seq_num, crc)

// Util Frame ID
#define CANMORE_CALC_UTIL_ID(client_id, direction, channel) CANMORE_CALC_STD_ID(client_id, CANMORE_TYPE_UTIL, direction, channel)
#define CANMORE_CALC_UTIL_ID_A2C(client_id, channel) CANMORE_CALC_UTIL_ID(client_id, CANMORE_DIRECTION_AGENT_TO_CLIENT, channel)
#define CANMORE_CALC_UTIL_ID_C2A(client_id, channel) CANMORE_CALC_UTIL_ID(client_id, CANMORE_DIRECTION_CLIENT_TO_AGENT, channel)

// Heartbeat Message
#define CANMORE_CALC_HEARTBEAT_DATA(cnt, error, extra) \
    ( \
        (((cnt) & ((1u<<CANMORE_HEARTBEAT_CNT_LENGTH) - 1u)) << CANMORE_HEARTBEAT_CNT_OFFSET) | \
        (((error) & ((1u<<CANMORE_HEARTBEAT_ERROR_LENGTH) - 1u)) << CANMORE_HEARTBEAT_ERROR_OFFSET) | \
        (((extra) & ((1u<<CANMORE_HEARTBEAT_EXTRA_LENGTH) - 1u)) << CANMORE_HEARTBEAT_EXTRA_OFFSET) \
    )


// ========================================
// Filter Macros
// ========================================
// Filter mask for standard ID frames
#define CANMORE_CALC_FILTER_MASK(match_client_id, match_type, match_direction, match_noc) \
    ( \
        ((match_client_id ? (1u<<CANMORE_CLIENT_ID_LENGTH) - 1u : 0u) << CANMORE_STD_CLIENT_ID_OFFSET) | \
        ((match_type ? (1u<<CANMORE_TYPE_LENGTH) - 1u : 0u) << CANMORE_STD_TYPE_OFFSET) | \
        ((match_direction ? (1u<<CANMORE_DIRECTION_LENGTH) - 1u : 0u) << CANMORE_STD_DIRECTION_OFFSET) | \
        ((match_noc ? (1u<<CANMORE_NOC_LENGTH) - 1u : 0u) << CANMORE_STD_NOC_OFFSET) \
    )

// Filter mask for extended ID message type frames
#define CANMORE_CALC_EXT_FILTER_MASK(match_client_id, match_type, match_direction, match_noc, match_crc) \
    ( \
        ((match_client_id ? (1u<<CANMORE_CLIENT_ID_LENGTH) - 1u : 0u) << CANMORE_CRC_CLIENT_ID_OFFSET) | \
        ((match_type ? (1u<<CANMORE_TYPE_LENGTH) - 1u : 0u) << CANMORE_CRC_TYPE_OFFSET) | \
        ((match_direction ? (1u<<CANMORE_DIRECTION_LENGTH) - 1u : 0u) << CANMORE_CRC_DIRECTION_OFFSET) | \
        ((match_noc ? (1u<<CANMORE_NOC_LENGTH) - 1u : 0u) << CANMORE_CRC_NOC_OFFSET) | \
        ((match_crc ? (1u<<CANMORE_CRC_LENGTH) - 1u : 0u) << CANMORE_CRC_CRC_OFFSET) \
    )

#ifdef __cplusplus
}
#endif

#endif