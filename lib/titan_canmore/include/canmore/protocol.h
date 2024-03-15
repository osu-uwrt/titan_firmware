#ifndef CANMORE__PROTOCOL_H_
#define CANMORE__PROTOCOL_H_

#include <assert.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @file canmore/protocol.h
 *
 * @brief CANmore Protocol Specification and Important Constants
 *
 */

/*
 * CANmore Specification
 * **********************
 * CANmore is a transport layer definition, addressing scheme, and protocol to allow bandwidth-efficient communication
 * on ISO 11898-1 CAN and ISO 11898-1:2015 CAN-FD networks carrying DDS-XRCE messages.
 *
 *
 * Purpose
 * =======
 * This specification is designed to build upon the CAN and CAN-FD.
 *
 * This speicfication is designed to allow DDS-XRCE communication with minimal framing overhead. Presently, XRCE-DDS has
 * two transport types: stream and packet. Packet transports send raw DDS-XRCE messages over the transport, but does not
 * allow fragmentation. In packet transport mode, if the MTU is less than the message size, the message will be dropped.
 * Due to CAN's 8-byte MTU, DDS-XRCE messages are unable to fit inside a single CAN frame. Stream transport is designed
 * to handle fragmentation, but adds 8 bytes of overhead for each DDS message. Due to the low-bandwidth available on CAN
 * bus, special attention must be paid to unnessary traffic on the bus to allow for maximum useful bandwidth on the bus.
 * Stream transport, additionally, has several fields unnecessary for CAN bus, such as synchronization, device ID, and
 * error correction fields, all of which are already present in the CAN specification.
 *
 * This specification allows using packet transport with minimal overhead by using the CAN frame ID field to contain the
 * required data to reassemble a fragmented packet, keeping the data fields for the DDS-XRCE message. Additionally, bits
 * are used to allow out-of-band traffic to be sent in parallel over the bus, such as diagnostics or firmware updates.
 *
 *
 * Definitions
 * ===========
 * Agent: The primary CAN node linking the clients into the ROS network. This is typically the main robot computer
 * Bus: A network of interconnected CAN nodes, with one agent at least one one client node
 * Client: A CAN node that primarily communicates with the agent. This is usually a microcontroller
 * Client ID: A unique ID assigned to each client on the bus
 * Channel: The utility frame channel number
 * Frame: An ISO 11898 CAN Bus Frame
 * Node: An ISO 11898 CAN compliant endpoint
 * Message: A single DDS-XRCE packet. This can be spread over several CAN frames
 * Message (Type) Frame: A CANmore frame type designed to carry fragmented messages
 * NOC: Number or Channel, This field represents the message sequence number or utility frame channel, depending on
 * frame type RFU: Reserved for Future Use. This field does not curerntly have a defined value, but could be defined in
 * the future. Should be set to 0 Sequence Number: The message frame sequence number
 *
 *
 * Bus Hierarchy
 * =============
 * This bus has a single central agent node with one or several client nodes. Each client is assigned a unique Client
 * ID. All message type communication on the network is between a client and the agent. Two clients cannot communicate
 * directly. This is required to avoid breaking the CAN specification, which prohibits two CAN bus nodes from
 * transmitting a frame with the same identifier. If CAN traffic must be sent between client nodes, then the next
 * level specification must allocate an agent to client utility channel for another client to use, and ensure that
 * no other clients nor the agent will attempt to transmit on that channel.
 *
 *                       +-------+
 *                     /-| ID 10 |
 *                   /-  +-------+
 *                 /-
 * +-----------+ /-      +-------+
 * |   Agent   |---------| ID 11 |
 * +-----------+ \-      +-------+
 *                 \-
 *                   \-  +-------+
 *                     \-| ID 12 |
 *                       +-------+
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
 * has a channel: 0-15, which allows for several out-of-band queues to be defined for client applications.
 *
 *
 * Frame Format
 * ============
 * All of the protocol and framing information is packed inside the identifier field of the CAN frame, allowing for all
 * data fields of the frame to be used by higher-level protocols, minimizing overhead. For a message type, every frame
 * is standard except the first and last frames, which is are extended frame. See below for the first and last frame
 * formats. Utility frames can use either standard or extended frames, where the last 18 bytes are extra and are
 * available to the higher level protocol.
 *
 * RTR frames are not supported, nor are frames with the DLC field set to 0.
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
 *   | CLIENT ID | T | D | NOC !=0 |                EXTRA                |  (Extended 29-bit ID)
 *   +-*-*-*-*-*-+-*-+-*-+-*-*-*-*-+-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-+
 *    28      24  23  22  21    18  17                                 0
 *
 * Message First Extended CAN Frame Identifier Format (First Frame in Message):
 *   +-*-*-*-*-*-+-*-+-*-+-*-*-*-*-+-*-+-*-*-*-*-*-*-+-*-*-*-*-*-*-*-*-*-*-*-+
 *   | CLIENT ID | T | D | NOC = 0 | S | MSG SUBTYPE |      MSG LENGTH       |  (Extended 29-bit ID)
 *   +-*-*-*-*-*-+-*-+-*-+-*-*-*-*-+-*-+-*-*-*-*-*-*-+-*-*-*-*-*-*-*-*-*-*-*-+
 *    28      24  23  22  21    18  17  16        11  10                   0
 *
 * Message Last Extended CAN Frame Identifier Format (Last Frame in Message):
 *   +-*-*-*-*-*-+-*-+-*-+-*-*-*-*-+-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-+
 *   | CLIENT ID | T | D | NOC !=0 |                 CRC                 |  (Extended 29-bit ID)
 *   +-*-*-*-*-*-+-*-+-*-+-*-*-*-*-+-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-+
 *    28      24  23  22  21    18  17                                 0
 *
 * CLIENT ID: The client this frame is intended for (if sent from the agent) or originating from (if sent from the
 *            client). Up to 31 clients supported (client ID 0 reserved). Due to CAN arbitration, lower client IDs have
 *            higher priority.
 * TYPE (T): Signifies the frame type. 0 for message type frames; 1 for utility type frames.
 * DIRECTION (D): The direction of the request. 0 for client to agent, 1 for agent to client
 * NOC: [Number or Channel] The sequence number (if message type) or communication channel (if utility type).
 * EXTRA: The 18-bit portion of the extended ID, available to the utility or message protocol for extra data.
 * SINGLE (S): Set to 1 if this message frame is only a single message (and to not expect a last packet)
 * MSG SUBTYPE: A 6-bit integer which can be used by higher level protcols to designate the type of this message
 * LENGTH: The total length of the canmore message about to be sent. Required for CAN FD to properly trim large packets
 *         as CAN FD does not allow long frames to have precise lengths.
 * CRC: A CRC-18 checksum to ensure the message is re-assembled properly. Only valid in message type frames.
 *
 *
 * Message Frame Protocol
 * ======================
 * Frames with the type set to message follow the message frame protocol. Messages allow a larger packet to be
 * fragmented into CAN frames, and reliably reassembled. If any frame in the message was corrupted, the entire message
 * is dropped. This protocol only gaurentees that if a message arrives, it is not corrupted. It is up to the higher
 * level protocol to handle dropped messages.
 *
 * Each transmitted message begins with a start packet, which has a sequence number of 0. This packet should synchronize
 * the receiver, causing it to discard a previous partially-received message. The message is fragmented into individual
 * message frames, with the sequence number incrementing for each frame. For non CAN-FD frames, after sequence number
 * 15, the number should roll back to 1, avoiding 0 as it is reserved for the start packet. This rollover is provided to
 * allow the 1024 byte limit to be reached. The last frame transmitted should be an extended frame, signaling to the
 * receiver that it is the last frame in the message sequence. The lower 18 bits of the extended frame ID should contain
 * the CRC-18 of the complete message. This reduces the liklihood that a corrupt message is received if the bus drops
 * packets from the end of a previous message and the start of the next message, resuming communication where the
 * sequence number of the next message lines up with the expected sequence number from the previous message. The CRC-18
 * uses a polynomial of 0x23979 with an initial value of 0x3FFFF. No final XOR is applied.
 *
 * The maximum size of any message should not exceed 1024 bytes, to reduce memory requirements on embedded devices
 * implementing the CANmore protocol. This is also the maximum limit for CAN-FD without a sequence number rollover (64
 * byte max MTU * 16 possible sequences)
 *
 *
 * Utility Frame Protocol
 * ======================
 * Frames with the type to utility can be used for implementation defined, out-of-band data between the agent and
 * clients. For client-to-client communication, the space of IDs occupied by the reserved client ID 0 should be used
 * instead. Each utility frame has a channel ID, which should be assigned by the implementer.
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
#define CANMORE_MAX_FRAME_SIZE 8
#define CANMORE_MAX_FD_FRAME_SIZE 64

// Identifier Field Lengths
#define CANMORE_EXTRA_LENGTH 18
#define CANMORE_MSG_LEN_LENGTH 11
#define CANMORE_MSG_SUBTYPE_LENGTH 6
#define CANMORE_MSG_SINGLE_LENGTH 1
#define CANMORE_NOC_LENGTH 4
#define CANMORE_DIRECTION_LENGTH 1
#define CANMORE_TYPE_LENGTH 1
#define CANMORE_CLIENT_ID_LENGTH 5

// Standard Identifier Offsets
#define CANMORE_STD_NOC_OFFSET 0
#define CANMORE_STD_DIRECTION_OFFSET (CANMORE_STD_NOC_OFFSET + CANMORE_NOC_LENGTH)
#define CANMORE_STD_TYPE_OFFSET (CANMORE_STD_DIRECTION_OFFSET + CANMORE_DIRECTION_LENGTH)
#define CANMORE_STD_CLIENT_ID_OFFSET (CANMORE_STD_TYPE_OFFSET + CANMORE_TYPE_LENGTH)

// Extended Identifier Offsets
#define CANMORE_EXT_EXTRA_OFFSET 0
#define CANMORE_EXT_MSG_LEN_OFFSET 0
#define CANMORE_EXT_MSG_SUBTYPE_OFFSET (CANMORE_EXT_MSG_LEN_OFFSET + CANMORE_MSG_LEN_LENGTH)
#define CANMORE_EXT_MSG_SINGLE_OFFSET (CANMORE_EXT_MSG_SUBTYPE_OFFSET + CANMORE_MSG_SUBTYPE_LENGTH)
static_assert(CANMORE_EXT_EXTRA_OFFSET + CANMORE_EXTRA_LENGTH ==
                  CANMORE_EXT_MSG_SINGLE_OFFSET + CANMORE_MSG_SINGLE_LENGTH,
              "Msg metadata must be same size as extra data");
#define CANMORE_EXT_NOC_OFFSET (CANMORE_EXT_EXTRA_OFFSET + CANMORE_EXTRA_LENGTH)
#define CANMORE_EXT_DIRECTION_OFFSET (CANMORE_EXT_NOC_OFFSET + CANMORE_NOC_LENGTH)
#define CANMORE_EXT_TYPE_OFFSET (CANMORE_EXT_DIRECTION_OFFSET + CANMORE_DIRECTION_LENGTH)
#define CANMORE_EXT_CLIENT_ID_OFFSET (CANMORE_EXT_TYPE_OFFSET + CANMORE_TYPE_LENGTH)

// Direction types
#define CANMORE_DIRECTION_CLIENT_TO_AGENT 0
#define CANMORE_DIRECTION_AGENT_TO_CLIENT 1

// Frame Types
#define CANMORE_TYPE_MSG 0
#define CANMORE_TYPE_UTIL 1

// Maximum possible message size in bytes supported by CANmore protocol
#define CANMORE_MAX_MSG_LENGTH 1024

// Heartbeat field lengths
#define CANMORE_CHAN_HEARTBEAT 15
#define CANMORE_HEARTBEAT_CNT_LENGTH 2
#define CANMORE_HEARTBEAT_ERROR_LENGTH 1
#define CANMORE_HEARTBEAT_EXTRA_LENGTH 5

// Heartbeat offsets
#define CANMORE_HEARTBEAT_CNT_OFFSET 0
#define CANMORE_HEARTBEAT_ERROR_OFFSET (CANMORE_HEARTBEAT_CNT_OFFSET + CANMORE_HEARTBEAT_CNT_LENGTH)
#define CANMORE_HEARTBEAT_EXTRA_OFFSET (CANMORE_HEARTBEAT_ERROR_OFFSET + CANMORE_HEARTBEAT_ERROR_LENGTH)

// ========================================
// Decoding Unions
// ========================================

typedef union __attribute__((__packed__)) canmore_id {
    uint32_t identifier;
    struct canmore_id_std {
        uint32_t noc : CANMORE_NOC_LENGTH;
        uint32_t direction : CANMORE_DIRECTION_LENGTH;
        uint32_t type : CANMORE_TYPE_LENGTH;
        uint32_t client_id : CANMORE_CLIENT_ID_LENGTH;
    } pkt_std;
    struct canmore_id_ext {
        uint32_t extra : CANMORE_EXTRA_LENGTH;
        uint32_t noc : CANMORE_NOC_LENGTH;
        uint32_t direction : CANMORE_DIRECTION_LENGTH;
        uint32_t type : CANMORE_TYPE_LENGTH;
        uint32_t client_id : CANMORE_CLIENT_ID_LENGTH;
    } pkt_ext;
    struct canmore_id_ext_start {
        uint32_t msg_len : CANMORE_MSG_LEN_LENGTH;
        uint32_t msg_subtype : CANMORE_MSG_SUBTYPE_LENGTH;
        uint32_t msg_single : CANMORE_MSG_SINGLE_LENGTH;
        uint32_t noc : CANMORE_NOC_LENGTH;
        uint32_t direction : CANMORE_DIRECTION_LENGTH;
        uint32_t type : CANMORE_TYPE_LENGTH;
        uint32_t client_id : CANMORE_CLIENT_ID_LENGTH;
    } pkt_ext_start;
} canmore_id_t;

typedef union __attribute__((__packed__)) canmore_heartbeat {
    uint8_t data;
    struct canmore_heartbeat_packet {
        uint8_t cnt : CANMORE_HEARTBEAT_CNT_LENGTH;
        uint8_t error : CANMORE_HEARTBEAT_ERROR_LENGTH;
        uint8_t extra : CANMORE_HEARTBEAT_EXTRA_LENGTH;
    } pkt;
} canmore_heartbeat_t;

// ========================================
// Calculation Macros
// ========================================

#define CANMORE_CALC_STD_ID(client_id, type, direction, noc)                                                           \
    ((((client_id) & ((1u << CANMORE_CLIENT_ID_LENGTH) - 1)) << CANMORE_STD_CLIENT_ID_OFFSET) |                        \
     (((type) & ((1u << CANMORE_TYPE_LENGTH) - 1u)) << CANMORE_STD_TYPE_OFFSET) |                                      \
     (((direction) & ((1u << CANMORE_DIRECTION_LENGTH) - 1u)) << CANMORE_STD_DIRECTION_OFFSET) |                       \
     (((noc) & ((1u << CANMORE_NOC_LENGTH) - 1u)) << CANMORE_STD_NOC_OFFSET))

#define CANMORE_CALC_EXT_ID(client_id, type, direction, noc, extra)                                                    \
    ((((client_id) & ((1u << CANMORE_CLIENT_ID_LENGTH) - 1u)) << CANMORE_EXT_CLIENT_ID_OFFSET) |                       \
     (((type) & ((1u << CANMORE_TYPE_LENGTH) - 1u)) << CANMORE_EXT_TYPE_OFFSET) |                                      \
     (((direction) & ((1u << CANMORE_DIRECTION_LENGTH) - 1u)) << CANMORE_EXT_DIRECTION_OFFSET) |                       \
     (((noc) & ((1u << CANMORE_NOC_LENGTH) - 1u)) << CANMORE_EXT_NOC_OFFSET) |                                         \
     (((extra) & ((1u << CANMORE_EXTRA_LENGTH) - 1u)) << CANMORE_EXT_EXTRA_OFFSET))

#define CANMORE_CAL_MSG_FIRST_EXTRA(single, len, subtype)                                                              \
    ((((len) & ((1u << CANMORE_MSG_LEN_LENGTH) - 1)) << CANMORE_EXT_MSG_LEN_OFFSET) |                                  \
     (((subtype) & ((1u << CANMORE_MSG_SUBTYPE_LENGTH) - 1)) << CANMORE_EXT_MSG_SUBTYPE_OFFSET) |                      \
     (((single ? 1 : 0) & ((1u << CANMORE_MSG_SINGLE_LENGTH) - 1)) << CANMORE_EXT_MSG_SINGLE_OFFSET))

// Message standard ID
#define CANMORE_CALC_MSG_ID(client_id, direction, seq_num)                                                             \
    CANMORE_CALC_STD_ID(client_id, CANMORE_TYPE_MSG, direction, seq_num)
#define CANMORE_CALC_MSG_ID_A2C(client_id, seq_num)                                                                    \
    CANMORE_CALC_MSG_ID(client_id, CANMORE_DIRECTION_AGENT_TO_CLIENT, seq_num)
#define CANMORE_CALC_MSG_ID_C2A(client_id, seq_num)                                                                    \
    CANMORE_CALC_MSG_ID(client_id, CANMORE_DIRECTION_CLIENT_TO_AGENT, seq_num)

// Message extended ID
#define CANMORE_CALC_MSG_EXT_ID(client_id, direction, seq_num, extra)                                                  \
    CANMORE_CALC_EXT_ID(client_id, CANMORE_TYPE_MSG, direction, seq_num, extra)
#define CANMORE_CALC_MSG_EXT_ID_A2C(client_id, seq_num, extra)                                                         \
    CANMORE_CALC_MSG_EXT_ID(client_id, CANMORE_DIRECTION_AGENT_TO_CLIENT, seq_num, extra)
#define CANMORE_CALC_MSG_EXT_ID_C2A(client_id, seq_num, extra)                                                         \
    CANMORE_CALC_MSG_EXT_ID(client_id, CANMORE_DIRECTION_CLIENT_TO_AGENT, seq_num, extra)

// Message First (Extended) ID
#define CANMORE_CALC_MSG_FIRST_ID(client_id, direction, seq_num, single, len, subtype)                                 \
    CANMORE_CALC_EXT_ID(client_id, CANMORE_TYPE_MSG, direction, seq_num,                                               \
                        CANMORE_CAL_MSG_FIRST_EXTRA(single, len, subtype))
#define CANMORE_CALC_MSG_FIRST_ID_A2C(client_id, seq_num, single, len, subtype)                                        \
    CANMORE_CALC_MSG_FIRST_ID(client_id, CANMORE_DIRECTION_AGENT_TO_CLIENT, seq_num, single, len, subtype)
#define CANMORE_CALC_MSG_FIRST_ID_C2A(client_id, seq_num, single, len, subtype)                                        \
    CANMORE_CALC_MSG_FIRST_ID(client_id, CANMORE_DIRECTION_CLIENT_TO_AGENT, seq_num, single, len, subtype)

// Util Frame ID
#define CANMORE_CALC_UTIL_ID(client_id, direction, channel)                                                            \
    CANMORE_CALC_STD_ID(client_id, CANMORE_TYPE_UTIL, direction, channel)
#define CANMORE_CALC_UTIL_ID_A2C(client_id, channel)                                                                   \
    CANMORE_CALC_UTIL_ID(client_id, CANMORE_DIRECTION_AGENT_TO_CLIENT, channel)
#define CANMORE_CALC_UTIL_ID_C2A(client_id, channel)                                                                   \
    CANMORE_CALC_UTIL_ID(client_id, CANMORE_DIRECTION_CLIENT_TO_AGENT, channel)

// Heartbeat Message
#define CANMORE_CALC_HEARTBEAT_DATA(cnt, error, extra)                                                                 \
    ((((cnt) & ((1u << CANMORE_HEARTBEAT_CNT_LENGTH) - 1u)) << CANMORE_HEARTBEAT_CNT_OFFSET) |                         \
     (((error) & ((1u << CANMORE_HEARTBEAT_ERROR_LENGTH) - 1u)) << CANMORE_HEARTBEAT_ERROR_OFFSET) |                   \
     (((extra) & ((1u << CANMORE_HEARTBEAT_EXTRA_LENGTH) - 1u)) << CANMORE_HEARTBEAT_EXTRA_OFFSET))

// ========================================
// Filter Macros
// ========================================
// Filter mask for standard ID frames
#define CANMORE_CALC_FILTER_MASK(match_client_id, match_type, match_direction, match_noc)                              \
    (((match_client_id ? (1u << CANMORE_CLIENT_ID_LENGTH) - 1u : 0u) << CANMORE_STD_CLIENT_ID_OFFSET) |                \
     ((match_type ? (1u << CANMORE_TYPE_LENGTH) - 1u : 0u) << CANMORE_STD_TYPE_OFFSET) |                               \
     ((match_direction ? (1u << CANMORE_DIRECTION_LENGTH) - 1u : 0u) << CANMORE_STD_DIRECTION_OFFSET) |                \
     ((match_noc ? (1u << CANMORE_NOC_LENGTH) - 1u : 0u) << CANMORE_STD_NOC_OFFSET))

// Filter mask for extended ID message type frames
#define CANMORE_CALC_EXT_FILTER_MASK(match_client_id, match_type, match_direction, match_noc, match_extra)             \
    (((match_client_id ? (1u << CANMORE_CLIENT_ID_LENGTH) - 1u : 0u) << CANMORE_EXT_CLIENT_ID_OFFSET) |                \
     ((match_type ? (1u << CANMORE_TYPE_LENGTH) - 1u : 0u) << CANMORE_EXT_TYPE_OFFSET) |                               \
     ((match_direction ? (1u << CANMORE_DIRECTION_LENGTH) - 1u : 0u) << CANMORE_EXT_DIRECTION_OFFSET) |                \
     ((match_noc ? (1u << CANMORE_NOC_LENGTH) - 1u : 0u) << CANMORE_EXT_NOC_OFFSET) |                                  \
     ((match_extra ? (1u << CANMORE_EXTRA_LENGTH) - 1u : 0u) << CANMORE_EXT_EXTRA_OFFSET))

#ifdef __cplusplus
}
#endif

#endif
