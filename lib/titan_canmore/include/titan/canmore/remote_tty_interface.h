#ifndef TITAN__CANMORE__REMOTE_TTY_INTERFACE_H_
#define TITAN__CANMORE__REMOTE_TTY_INTERFACE_H_

#include "titan/canmore/protocol.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Remote TTY Interface
 * ********************
 * This interface exposes a TTY (or PTY) over CAN bus to allow accessing a debug shell over CAN bus.
 *
 * In this document the Client refers to the device which has the terminal that user is typing into, and the Host refers
 * to the device that the terminal processes (such as bash) are running on.
 *
 *
 * Allocated Subchannels
 * =====================
 * Due to the stream based nature of a TTY, several subchannels were allocated to the Remote TTY to allow for a
 * low-latency, low-overhead connection between the devices. All of these subchannels share a single canmore utlity
 * channel (see CAN ID Format for more information).
 *
 * There are two types of subchannels defined: Control and Stream channels.
 *
 * Four subchannels are created. The first channel is a control channel, while the rest are stream channels:
 *  - Control Channel (0) [Bidirectional]: Sends realtime control signals, such as acks, disconnects, and window resize
 *  - Stdin Channel (1) [Client to Host]: Stream for terminal stdin from the client
 *  - Stdout Channel (2) [Host to Client]: Stream for terminal stdout from the host
 *  - Stderr Channel (3) [Host to Client]: Stream for sending stderr from the host
 *
 *
 * Stream Channels
 * ===============
 * The goal of the stream channel is to abstract the unreliable CAN bus interface into a reliable stream, which
 * gaurentees *all* packets eventually arrive in-order and uncorrupted.
 *
 * In this section the word transmitter and receiver are used. Both the host and client can take on these roles (for
 * either the stdout/stderr or stdin streams).
 *
 * CAN bus does some of the heavy lifting for us here, where it gaurentees that individual packets will arrive intact,
 * however it makes no gaurentees that a packet will arrive at its intended destination. The stream channel then uses
 * the extended ID to add a sequence number, to ensure that all packets are processed in order. The stream channel
 * also uses the sequence number as part of an acknowldgement procedure to ensure that all data arrived at the
 * destination.
 *
 * Stream Acknowledgements
 * -----------------------
 *
 * Acknowledges are sent periodically from the receiver, after either a certain amount of time has passed or a certain
 * number of packets have been received. This reduces the overhead of acknowledges (as not every packet on the stream
 * channel requires an acknowledge). The max number of packets between an ack is determined by the transmitter's local
 * buffer size.
 *
 * Each receiver keeps track of the last received sequence number. It will only accept the next number in the sequence.
 * If a packet arrives which does not match the expected sequence number, it will be dropped.
 *
 * Each transmitter will hold a buffer containing all unacknowledged packets that have been sent over the bus. When the
 * receiver sends an acknowledge (over the control channel), the transmitter is free to delete all packets in its local
 * buffer up to the acknowledged sequence number. The transmitter will not read from the backing file descriptor feeding
 * the stream (such as the terminal STDIN_FILENO in the client case) if its local unacknowledged buffer fills up. This
 * blocks the upstream file descriptor, ensuring that no data is lost if the receiver looses connection for an extended
 * period of time.
 *
 * During normal operation, as packets arrive, the receiver will send an acknowledgement every n packets, where n is
 * ideally less than the size of the unacknowledged packet buffer. Additionally, if a given amount of time passes
 * without a VALID packet being received, the receiver will re-transmit the ack. This handles the case where the ack
 * gets dropped in transit.
 *
 * If the transmitter receives an ack matching the last ack value, it will begin retransmitting from its local
 * unacknowledged buffer. This ensures that if a single packet gets lost in transit, then the receiver will not receive
 * a valid packet (as it will never receive the expected sequence number). After a given amount of time passes on the
 * receiver, it will retransmit the ack, causing the transmitter to retransmit its local buffer, recovering from the
 * dropped packet.
 *
 * If the transmitter receives an ack for a sequence number not in its local unacknowledged buffer, it will consider
 * that the receiver has lost its internal state (as this should not occur during normal operation). In this case, the
 * transmitter will clear its internal unacknowledged buffer, and set the last acked sequence number to the value in the
 * ack packet from the receiver.
 *
 * The sequence number of 0 is special, as it will never be a sequence number for a transmitted stream packet. Instead,
 * if a receiver sends an ack packet with the sequence number of 0, this ensures that the transmitter will reset its
 * local unacknowledged buffer. This can be used when the receiver connects to a transmitter which is already running
 * to avoid getting old buffers. Additionally, the transmitter can wait for a sequence number of 0 ack before
 * sending the first stream packet to ensure that a receiver is listening on first startup.
 *
 * Note that all stream channels share a unique sequence number for a given direction. This is done to reduce the number
 * of acknowledge packets that need to be sent from a receiver. That means that stdout and stderr are based on the same
 * sequence number, and stdout will never have a packet with the same sequence number as stderr.
 * If packets need to be retransmitted from a given sequence number, the transmitter needs to ensure that the packets go
 * to the correct destination.
 *
 * Rollovers are handled by resetting the sequence number back to 1. It is up to the transmitter and receiver code to
 * properly implement logic to handle rollovers correctly. The number of unacknowledged packets in flight should be
 * sized such that there isn't ambiguity if a given sequence number is in the past or future.
 *
 *
 * CAN ID Format
 * =============
 *
 * Remote TTY Frame Format:
 *   +-*-*-*-*-*-+-*-+-*-+-*-*-*-*-+-*-*-+-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-+
 *   | CLIENT ID | T | D |   NOC   | SCH |             SEQ/CMD             |  (Extended 29-bit ID)
 *   +-*-*-*-*-*-+-*-+-*-+-*-*-*-*-+-*-*-+-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-+
 *    28      24  23  22  21    18    16  15                             0
 *
 * CLIENT ID, T, D, NOC - Same as Canmore Protocol
 * SUBCHANNEL (SCH): The subchannel
 *  - 0: CONTROL (Control Type)
 *  - 1: STDIN (Stream Type)
 *  - 2: STDOUT (Stream Type)
 *  - 3: STDERR (Stream Type)
 * SEQ/CMD: The sequence number if stream type channel, or the command if control type channel.
 *
 *
 * Control Channel Commands
 * ========================
 *
 * Command 0: Disconnect
 * ---------------------
 *
 * Notifies that the other side of the connection has disconnected. This command is supported by both the host and
 * client. If either side receives this command, it should tear down the connection and not expect any future packets to
 * be acknowledged.
 *
 * Data Format (DLC = 1):
 *   +--------+
 *   | Byte 0 |
 *   +--------+
 *   | Is Err |
 *   +--------+
 *
 * Is Err: Set to 1 if the disconnection is due to an error.
 *
 *
 * Command 1: Acknowledge
 * ----------------------
 *
 * Acknowledges packets received from the transmitter. This command is supported by both the host and client. Upon
 * initial connection, this should be sent with a Last Seq Num of 0 to signal the initial connection.
 *
 * Data Format (DLC = 2):
 *   +--------+--------+
 *   | Byte 0 | Byte 1 |
 *   +--------+--------+
 *   |  Last Seq Num   |
 *   +--------+--------+
 *
 * Last Seq Num: The last sequence number received by the receiver.
 *               Because 0 is never a valid sequence number, setting this to 0 will clear the unacknowledged buffer.
 *
 *
 * Command 2: Window Size
 * ----------------------
 *
 * Updates the host with the current window size on the client. This command is only supported from the client.
 *
 * Data Format (DLC = 4):
 *   +--------+--------+--------+--------+
 *   | Byte 0 | Byte 1 | Byte 2 | Byte 3 |
 *   +--------+--------+--------+--------+
 *   |      Rows       |       Cols      |
 *   +-----------------+-----------------+
 *
 * Rows: The number of rows in the new window size
 * Cols: The number of columns in the new window size
 *
 */

// ========================================
// Remote TTY Stream Default Definitions
// ========================================

/**
 * @brief The maximum number of unacknowledged packets that can be in flight at once (Stream Transmitter Parameter)
 */
#define CANMORE_REMOTE_TTY_DEFAULT_MAX_IN_FLIGHT 24

/**
 * @brief Transmits an ack when this many packets have been successfully received since an ack has last been sent. This
 * should ideally be less than the transmitter's MAX_IN_FLIGHT value. (Stream Transmitter Parameter)
 */
#define CANMORE_REMOTE_TTY_DEFAULT_RX_PACKETS_PER_ACK 16

/**
 * @brief The time in ms since a packet was received before an ack is sent out (Stream Receiver Parameter)
 */
#define CANMORE_REMOTE_TTY_DEFAULT_ACK_INTERVAL_MS 500

// ========================================
// Remote TTY ID Definitions
// ========================================

// Define lengths as stated above
#define CANMORE_REMOTE_TTY_ID_SEQ_CMD_LENGTH 16
#define CANMORE_REMOTE_TTY_ID_SUBCH_LENGTH 2

// Compute offsets using ordering from above
#define CANMORE_REMOTE_TTY_ID_SEQ_CMD_OFFSET 0
#define CANMORE_REMOTE_TTY_ID_SUBCH_OFFSET (CANMORE_REMOTE_TTY_ID_SEQ_CMD_OFFSET + CANMORE_REMOTE_TTY_ID_SEQ_CMD_LENGTH)

// Define Subchannel Macros
#define CANMORE_REMOTE_TTY_SUBCH_CONTROL 0
#define CANMORE_REMOTE_TTY_SUBCH_STDIN 1
#define CANMORE_REMOTE_TTY_SUBCH_STDOUT 2
#define CANMORE_REMOTE_TTY_SUBCH_STDERR 3

// Verify our math
static_assert(CANMORE_EXT_NOC_OFFSET == CANMORE_EXTRA_LENGTH, "Canmore API changed from what I expected");
static_assert(CANMORE_REMOTE_TTY_ID_SEQ_CMD_LENGTH + CANMORE_REMOTE_TTY_ID_SUBCH_LENGTH == CANMORE_EXTRA_LENGTH,
              "Failed expected packing, the CRC macros below will be wrong");
#define CANMORE_REMOTE_TTY_CALC_EXTRA_FIELD_FOR_ID(subch, seq_cmd)                                                     \
    (((subch) & ((1u << CANMORE_REMOTE_TTY_ID_SUBCH_LENGTH) - 1u)) << CANMORE_REMOTE_TTY_ID_SUBCH_OFFSET) |            \
        (((seq_cmd) & ((1u << CANMORE_REMOTE_TTY_ID_SEQ_CMD_LENGTH) - 1u)) << CANMORE_REMOTE_TTY_ID_SEQ_CMD_OFFSET)

// Macros for computing IDs
#define CANMORE_REMOTE_TTY_CALC_ID(client_id, direction, subch, seq_cmd)                                               \
    CANMORE_CALC_EXT_ID(client_id, CANMORE_TYPE_UTIL, direction, CANMORE_TITAN_CHAN_REMOTE_TTY,                        \
                        CANMORE_REMOTE_TTY_CALC_EXTRA_FIELD_FOR_ID(subch, seq_cmd))
#define CANMORE_REMOTE_TTY_CALC_ID_A2C(client_id, subch, seq_cmd)                                                      \
    CANMORE_REMOTE_TTY_CALC_ID(client_id, CANMORE_DIRECTION_AGENT_TO_CLIENT, subch, seq_cmd)
#define CANMORE_REMOTE_TTY_CALC_ID_C2A(client_id, subch, seq_cmd)                                                      \
    CANMORE_REMOTE_TTY_CALC_ID(client_id, CANMORE_DIRECTION_CLIENT_TO_AGENT, subch, seq_cmd)

// ID Mask: Match everything except for the Remote TTY data in the rest of the packet
#define CANMORE_REMOTE_TTY_ID_MASK CANMORE_CALC_EXT_FILTER_MASK(1, 1, 1, 1, 0)

// ========================================
// Control Channel Commands
// ========================================

#define CANMORE_REMOTE_TTY_CMD_DISCONNECT_ID 0
#define CANMORE_REMOTE_TTY_CMD_DISCONNECT_LEN 1
#define CANMORE_REMOTE_TTY_CMD_ACK_ID 1
#define CANMORE_REMOTE_TTY_CMD_ACK_LEN 2
#define CANMORE_REMOTE_TTY_CMD_WINDOW_SIZE_ID 2
#define CANMORE_REMOTE_TTY_CMD_WINDOW_SIZE_LEN 4

typedef union canmore_remote_tty_cmd_disconnect {
    uint8_t data[CANMORE_REMOTE_TTY_CMD_DISCONNECT_LEN];
    struct __attribute__((packed)) {
        bool is_err;
    } pkt;
} canmore_remote_tty_cmd_disconnect_t;
static_assert(sizeof(canmore_remote_tty_cmd_disconnect_t) == CANMORE_REMOTE_TTY_CMD_DISCONNECT_LEN, "Failed to pack");

typedef union canmore_remote_tty_cmd_ack {
    uint8_t data[CANMORE_REMOTE_TTY_CMD_ACK_LEN];
    struct __attribute__((packed)) {
        uint16_t last_seq_num;
    } pkt;
} canmore_remote_tty_cmd_ack_t;
static_assert(sizeof(canmore_remote_tty_cmd_ack_t) == CANMORE_REMOTE_TTY_CMD_ACK_LEN, "Failed to pack");

typedef union canmore_remote_tty_cmd_window_size {
    uint8_t data[CANMORE_REMOTE_TTY_CMD_WINDOW_SIZE_LEN];
    struct __attribute__((packed)) {
        uint16_t rows;
        uint16_t cols;
    } pkt;
} canmore_remote_tty_cmd_window_size_t;
static_assert(sizeof(canmore_remote_tty_cmd_window_size_t) == CANMORE_REMOTE_TTY_CMD_WINDOW_SIZE_LEN, "Failed to pack");

#ifdef __cplusplus
}
#endif

#endif
