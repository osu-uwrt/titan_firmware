#ifndef CANMORE__MSG_ENCODING_H
#define CANMORE__MSG_ENCODING_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "canmore/protocol.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @file canmore_msg_encoding.h
 *
 * API for encoding/decoding CANmore message type frames
 *
 * This provides an encoder and decoder which are able to convert between raw message buffers and the CAN frames to be
 * sent over the CAN bus compliant to the CANmore protocol. See CANmore protocol specification header file for more
 * information.
*/

/**
 * @brief Maximum message sequence number supported by CANmore protocol
*/
#define CANMORE_MAX_MSG_SEQ_NUM  (1<<CANMORE_NOC_LENGTH)

/**
 * @brief Maximum possible message size in bytes supported by CANmore protocol
*/
#define CANMORE_MAX_MSG_LENGTH  (CANMORE_MAX_MSG_SEQ_NUM * CANMORE_FRAME_SIZE)


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
    // The buffer containing the data to be fragmented into frames
    uint8_t buffer[CANMORE_MAX_MSG_LENGTH];
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
 * @param direction The direction to be used when encoding frame IDs (CANMORE_DIRECTION_CLIENT_TO_AGENT / CANMORE_DIRECTION_AGENT_TO_CLIENT)
*/
void canmore_msg_encode_init(canmore_msg_encoder_t *state, uint32_t client_id, uint32_t direction);

/**
 * @brief Loads a new message into the CANmore encoder
 * If a previous message has not been fully flushed, it will be discarded and overwritten
 *
 * @param state Pointer to encoder state data struct
 * @param buffer Buffer containing message data to encode
 * @param len Length of message data to encode
*/
void canmore_msg_encode_load(canmore_msg_encoder_t *state, const uint8_t *buffer, size_t len);

/**
 * @brief Checks if any remaining frames exist in the encoder to be sent
 *
 * @param state Pointer to encoder state data struct
 * @return True if data still needs to be sent, false if no data remaining to be transmitted
*/
bool canmore_msg_encode_done(canmore_msg_encoder_t *state);

/**
 * @brief Encodes the next frame to be sent from the message loaded from the `canmore_msg_encode_load` call
 *
 * @param state Pointer to encoder state data struct
 * @param buffer_out Buffer to write next frame data into (must be at least CANMORE_FRAME_SIZE bytes in size)
 * @param dlc_out Pointer to write the frame dlc (length of data written into buffer_out)
 * @param id_out Pointer to write the frame ID to be transmitted
 * @param is_extended Pointer to write boolean, true if the frame ID written to `id_out` is an extended ID
 * @return True if frame was successfully encoded, false if otherwise
*/
bool canmore_msg_encode_next(canmore_msg_encoder_t *state, uint8_t *buffer_out, uint8_t *dlc_out, uint32_t *id_out, bool *is_extended);


// ========================================
// CANmore Message Decoder
// ========================================

/**
 * @brief Decoder error constant definitions
 */
#define CANMORE_MSG_DECODER_ERROR_BAD_SEQ_NUM 0
#define CANMORE_MSG_DECODER_ERROR_ZERO_LENGTH_PACKET 1
#define CANMORE_MSG_DECODER_ERROR_BUFFER_OVERFLOW 2
#define CANMORE_MSG_DECODER_ERROR_CRC_FAIL 3

/**
 * @brief Callback for reporting a decoder error
 */
typedef void (*canmore_msg_decoder_error_handler_t)(void* arg, unsigned int error_code);

/**
 * @brief Struct containing decoder state
 *
 * Do not modify this struct directly! Use the `canmore_msg_decode_` set of functions instead
*/
typedef struct canmore_msg_decoder_state {
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
    void* decode_error_arg;
} canmore_msg_decoder_t;

/**
 * @brief Initializes a new CANmore message decoder
 *
 * @param state Pointer to decoder state struct to store internal state
 * @param decode_error_handler Callback for reporting a decoder error, can be NULL if no handler assigned
 * @param decode_error_arg Argument to provide to decode error handler
*/
void canmore_msg_decode_init(canmore_msg_decoder_t *state, canmore_msg_decoder_error_handler_t decode_error_handler, void* decode_error_arg);

/**
 * @brief Resets the message decoder state to receive a new sequence of message frames
 * @param state Pointer to decoder state data struct
*/
void canmore_msg_decode_reset_state(canmore_msg_decoder_t *state);

/**
 * @brief Add a new frame to the decoder state
 * Note that the last frame should be decoded using canmore_msg_decode_last_frame
 *
 * @param state Pointer to decoder state data struct
 * @param seq_num The frame sequence number decoded from the frame ID
 * @param data Buffer containing frame data
 * @param data_len Length of data in buffer
 * @return True if the frame was successfully decoded, false if error occurred
*/
bool canmore_msg_decode_frame(canmore_msg_decoder_t *state, uint8_t seq_num, uint8_t *data, size_t data_len);

/**
 * @brief Decode the last frame in a CANmore message transmission (extended frame)
 *
 * @param state Pointer to decoder state data struct
 * @param seq_num The frame sequence number decoded from the extended frame ID
 * @param data Buffer containing frame data
 * @param data_len Length of data in buffer
 * @param crc The crc decoded from the extended frame ID
 * @param data_out Buffer to copy decoded message into (must be at least CANMORE_MAX_MSG_LENGTH bytes in size)
 * @return Length of data copied into data_out
*/
size_t canmore_msg_decode_last_frame(canmore_msg_decoder_t *state, uint8_t seq_num, uint8_t *data, size_t data_len, uint32_t crc, uint8_t *data_out);

#ifdef __cplusplus
}
#endif

#endif