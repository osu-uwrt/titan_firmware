#ifndef CANMORE__MSG_ENCODING_H_
#define CANMORE__MSG_ENCODING_H_

#include "canmore/protocol.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @file canmore/canmore_msg_encoding.h
 *
 * @brief API for encoding/decoding CANmore message type frames
 *
 * This provides an encoder and decoder which are able to convert between raw message buffers and the CAN frames to be
 * sent over the CAN bus compliant to the CANmore protocol. See CANmore protocol specification header file for more
 * information.
 */

/**
 * @brief Maximum message sequence number supported by CANmore protocol
 */
#define CANMORE_MAX_MSG_SEQ_NUM ((1 << CANMORE_NOC_LENGTH) - 1)

// ========================================
// CANmore Message Encoder
// ========================================

/**
 * @brief Struct containing encoder state
 *
 * Do not modify this struct directly! Use the `canmore_msg_encode_` set of functions instead
 */
typedef struct canmore_msg_encoder_state {
    // The client id to be used when encoding frame IDs
    uint32_t client_id;
    // The direction to be used when encoding frame IDs
    uint32_t direction;
    // The subtype for the message being encoded
    uint8_t subtype;
    // Setting this bool enables CAN FD, increasing the max transmit size for each frame
    bool use_canfd;
    // The pointer to the buffer containing the data. This must be vaild from when load is called to when returns true
    const uint8_t *buf_ptr;
    // The next sequence number to be used when encoding frame IDs
    uint32_t seq_num;
    // The length of data in `buffer`
    size_t length;
    // The position of the next byte to send out from `buffer`
    size_t position;
} canmore_msg_encoder_t;

/**
 * @brief Initialize a new CANmore message encoder
 *
 * @param state Pointer to decoder state struct to store internal state
 * @param client_id The client id to be used when encoding frame IDs
 * @param direction The direction to be used when encoding frame IDs (CANMORE_DIRECTION_CLIENT_TO_AGENT /
 * CANMORE_DIRECTION_AGENT_TO_CLIENT)
 * @param use_canfd Enables CAN FD mode, expanding the max frame size from the standard can frame size to CAN FD
 */
void canmore_msg_encode_init(canmore_msg_encoder_t *state, uint32_t client_id, uint32_t direction, bool use_canfd);

/**
 * @brief Loads a new message into the CANmore encoder
 * If a previous message has not been fully flushed, it will be discarded and overwritten
 *
 * @attention The buffer pointer is copied to the encoder, rather than the contents of buffer. This means that buffer
 * must remain valid from when this function is called until canmore_msg_encode_done returns true. If this function is
 * serviced asynchronously, such as by interrupts or timers, the caller must copy the data to a separate buffer which
 * remains valid for the lifetime of this encoder object.
 *
 * @param state Pointer to encoder state data struct
 * @param subtype The subtype for this message to encode (6-bit value which is forwarded with the message)
 * @param buffer Buffer containing message data to encode
 * @param len Length of message data to encode
 */
void canmore_msg_encode_load(canmore_msg_encoder_t *state, uint8_t subtype, const uint8_t *buffer, size_t len);

/**
 * @brief Checks if any remaining frames exist in the encoder to be sent
 *
 * @note After this function returns true, it is safe to release the buffer pointer passed to canmore_msg_encode_load.
 *
 * @param state Pointer to encoder state data struct
 * @return True if data still needs to be sent, false if no data remaining to be transmitted
 */
bool canmore_msg_encode_done(canmore_msg_encoder_t *state);

/**
 * @brief Encodes the next frame to be sent from the message loaded from the `canmore_msg_encode_load` call
 *
 * @note This function will handle the padding of the length for CAN FD. For example, if the last packet is 29 bytes,
 * the CAN FD packet will need to be 32 bytes long (the next closest DLC). This function will write the 29 byte packet,
 * followed by 3 bytes of 0s. len_out will be written as 32.
 *
 * @attention buffer_out must be at least CANMORE_MAX_FRAME_SIZE bytes if use_canfd is configured to false, or
 * CANMORE_MAX_FD_FRAME_SIZE if use_canfd is configured to true.
 *
 * @param state Pointer to encoder state data struct
 * @param buffer_out Buffer to write next frame data into
 * @param len_out Pointer to write length of data written into buffer_out
 * @param id_out Pointer to write the frame ID to be transmitted
 * @param is_extended Pointer to write boolean, true if the frame ID written to `id_out` is an extended ID
 * @return True if frame was successfully encoded, false if otherwise
 */
bool canmore_msg_encode_next(canmore_msg_encoder_t *state, uint8_t *buffer_out, uint8_t *len_out, uint32_t *id_out,
                             bool *is_extended);

// ========================================
// CANmore Message Decoder
// ========================================

/**
 * @brief Decoder error constant definitions
 */
#define CANMORE_MSG_DECODER_ERROR_BAD_SEQ_NUM 0
#define CANMORE_MSG_DECODER_ERROR_ZERO_LENGTH_PACKET 1
#define CANMORE_MSG_DECODER_ERROR_UNEXPECTED_DATA 2
#define CANMORE_MSG_DECODER_ERROR_CRC_FAIL 3
#define CANMORE_MSG_DECODER_ERROR_SEQ_ZERO_NOT_EXTENDED 4
#define CANMORE_MSG_DECODER_ERROR_MSG_TOO_LARGE 5
#define CANMORE_MSG_DECODER_ERROR_MSG_TOO_SMALL 6
#define CANMORE_MSG_DECODER_ERROR_ROLLOVER_WITH_CANFD 7
#define CANMORE_MSG_DECODER_ERROR_INVALID_CLIENT_ID 8

/**
 * @brief Callback for reporting a decoder error
 */
typedef void (*canmore_msg_decoder_error_handler_t)(void *arg, unsigned int error_code);

/**
 * @brief Struct containing decoder state
 *
 * Do not modify this struct directly! Use the `canmore_msg_decode_` set of functions instead
 */
typedef struct canmore_msg_decoder_state {
    // Setting this bool enables CAN FD, increasing the max transmit size for each frame
    bool use_canfd;
    // Holds the subtype for this message received in the first packet
    uint8_t subtype;
    // Holds the expected length of the packet received in the first frame
    size_t expected_len;
    // Current crc18 decode value
    uint32_t crc18;
    // The next seq_num expected when receiving data
    uint32_t next_seq_num;
    // The size of data contained in the decode buffer
    size_t decode_len;
    // Buffer containing partially decode canmore message
    uint8_t decode_buffer[CANMORE_MAX_MSG_LENGTH];
    // Decoder error handler, can be NULL if no handler assigned
    canmore_msg_decoder_error_handler_t decode_error_handler;
    // Decoder error handler optional argument
    void *decode_error_arg;
} canmore_msg_decoder_t;

/**
 * @brief Initializes a new CANmore message decoder
 *
 * @param state Pointer to decoder state struct to store internal state
 * @param decode_error_handler Callback for reporting a decoder error, can be NULL if no handler assigned
 * @param decode_error_arg Argument to provide to decode error handler
 * @param use_canfd Enables CAN FD mode, expanding the max frame size from the standard can frame size to CAN FD
 */
void canmore_msg_decode_init(canmore_msg_decoder_t *state, canmore_msg_decoder_error_handler_t decode_error_handler,
                             void *decode_error_arg, bool use_canfd);

/**
 * @brief Resets the message decoder state to receive a new sequence of message frames
 * @param state Pointer to decoder state data struct
 */
void canmore_msg_decode_reset_state(canmore_msg_decoder_t *state);

/**
 * @brief Add a new frame to the decoder state
 *
 * @note To reduce the overhead of copying, the decoder keeps the last decoded message state in a buffer. After this
 * function returns a non-zero value, to retreive the decoded message, call the canmore_msg_decode_get_* functions to
 * retrieve the received message
 *
 * @param state Pointer to decoder state data struct
 * @param can_id The CAN frame ID for this frame
 * @param is_extended True if can_id is an extended frame id
 * @param frame Buffer containing frame data
 * @param frame_len Length of the incoming CAN frame
 * @return Length of the decoded message (read with canmore_msg_decode_get_buf), or 0 if no message finished decoding
 */
size_t canmore_msg_decode_frame(canmore_msg_decoder_t *state, uint32_t can_id, bool is_extended, const uint8_t *frame,
                                size_t frame_len);

/**
 * @brief Returns the subtype of the last decoded message.
 *
 * @attention canmore_msg_decode_frame must return a non-zero value before calling this function. Otherwise the data
 * returned is undefined.
 *
 * @param state Pointer to decoder state data struct
 * @return uint8_t The subtype for the last decoded message
 */
static inline uint8_t canmore_msg_decode_get_subtype(canmore_msg_decoder_t *state) {
    return state->subtype;
}

/**
 * @brief Returns pointer to the last decoded message.
 *
 * @note The length of the data in this buffer is the value returned by the last call to canmore_msg_decode_frame
 *
 * @attention canmore_msg_decode_frame must return a non-zero value before calling this function. Otherwise the data
 * returned is undefined.
 *
 * @param state Pointer to decoder state data struct
 * @return uint8_t* Pointer to the last decoded message
 */
static inline uint8_t *canmore_msg_decode_get_buf(canmore_msg_decoder_t *state) {
    return state->decode_buffer;
}

/**
 * @brief Converts the given length to a the next available CANFD DLC value which can hold the length
 *
 * @note If length is > the max CAN FD frame size, the max DLC of 0xF will be returned
 *
 * @param len The length of the message in bytes
 * @return unsigned char The smallest CAN FD DLC which can hold the length
 */
uint8_t canmore_fd_len2dlc(uint8_t len);

/**
 * @brief Converts the given CANFD DLC value to its corresponing length in bytes
 *
 * @param dlc The CAN FD dlc value. MUST BE CLAMPED TO 4-bit
 * @return uint8_t The message length for the given DLC
 */
uint8_t canmore_fd_dlc2len(uint8_t dlc);

#ifdef __cplusplus
}
#endif

#endif
