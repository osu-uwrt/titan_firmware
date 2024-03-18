#include "canmore/msg_encoding.h"

#include <string.h>

// ========================================
// CRC-18 Calculations
// ========================================

#define CRC18_INITIAL_VALUE 0x3FFFF
#define CRC18_MASK 0x3FFFF

static const uint32_t crc18_lookup[] = {
    0x00000, 0x23979, 0x24b8b, 0x072f2, 0x2ae6f, 0x09716, 0x0e5e4, 0x2dc9d, 0x365a7, 0x15cde, 0x12e2c, 0x31755, 0x1cbc8,
    0x3f2b1, 0x38043, 0x1b93a, 0x0f237, 0x2cb4e, 0x2b9bc, 0x080c5, 0x25c58, 0x06521, 0x017d3, 0x22eaa, 0x39790, 0x1aee9,
    0x1dc1b, 0x3e562, 0x139ff, 0x30086, 0x37274, 0x14b0d, 0x1e46e, 0x3dd17, 0x3afe5, 0x1969c, 0x34a01, 0x17378, 0x1018a,
    0x338f3, 0x281c9, 0x0b8b0, 0x0ca42, 0x2f33b, 0x02fa6, 0x216df, 0x2642d, 0x05d54, 0x11659, 0x32f20, 0x35dd2, 0x164ab,
    0x3b836, 0x1814f, 0x1f3bd, 0x3cac4, 0x273fe, 0x04a87, 0x03875, 0x2010c, 0x0dd91, 0x2e4e8, 0x2961a, 0x0af63, 0x3c8dc,
    0x1f1a5, 0x18357, 0x3ba2e, 0x166b3, 0x35fca, 0x32d38, 0x11441, 0x0ad7b, 0x29402, 0x2e6f0, 0x0df89, 0x20314, 0x03a6d,
    0x0489f, 0x271e6, 0x33aeb, 0x10392, 0x17160, 0x34819, 0x19484, 0x3adfd, 0x3df0f, 0x1e676, 0x05f4c, 0x26635, 0x214c7,
    0x02dbe, 0x2f123, 0x0c85a, 0x0baa8, 0x283d1, 0x22cb2, 0x015cb, 0x06739, 0x25e40, 0x082dd, 0x2bba4, 0x2c956, 0x0f02f,
    0x14915, 0x3706c, 0x3029e, 0x13be7, 0x3e77a, 0x1de03, 0x1acf1, 0x39588, 0x2de85, 0x0e7fc, 0x0950e, 0x2ac77, 0x070ea,
    0x24993, 0x23b61, 0x00218, 0x1bb22, 0x3825b, 0x3f0a9, 0x1c9d0, 0x3154d, 0x12c34, 0x15ec6, 0x367bf, 0x1a8c1, 0x391b8,
    0x3e34a, 0x1da33, 0x306ae, 0x13fd7, 0x14d25, 0x3745c, 0x2cd66, 0x0f41f, 0x086ed, 0x2bf94, 0x06309, 0x25a70, 0x22882,
    0x011fb, 0x15af6, 0x3638f, 0x3117d, 0x12804, 0x3f499, 0x1cde0, 0x1bf12, 0x3866b, 0x23f51, 0x00628, 0x074da, 0x24da3,
    0x0913e, 0x2a847, 0x2dab5, 0x0e3cc, 0x04caf, 0x275d6, 0x20724, 0x03e5d, 0x2e2c0, 0x0dbb9, 0x0a94b, 0x29032, 0x32908,
    0x11071, 0x16283, 0x35bfa, 0x18767, 0x3be1e, 0x3ccec, 0x1f595, 0x0be98, 0x287e1, 0x2f513, 0x0cc6a, 0x210f7, 0x0298e,
    0x05b7c, 0x26205, 0x3db3f, 0x1e246, 0x190b4, 0x3a9cd, 0x17550, 0x34c29, 0x33edb, 0x107a2, 0x2601d, 0x05964, 0x02b96,
    0x212ef, 0x0ce72, 0x2f70b, 0x285f9, 0x0bc80, 0x105ba, 0x33cc3, 0x34e31, 0x17748, 0x3abd5, 0x192ac, 0x1e05e, 0x3d927,
    0x2922a, 0x0ab53, 0x0d9a1, 0x2e0d8, 0x03c45, 0x2053c, 0x277ce, 0x04eb7, 0x1f78d, 0x3cef4, 0x3bc06, 0x1857f, 0x359e2,
    0x1609b, 0x11269, 0x32b10, 0x38473, 0x1bd0a, 0x1cff8, 0x3f681, 0x12a1c, 0x31365, 0x36197, 0x158ee, 0x0e1d4, 0x2d8ad,
    0x2aa5f, 0x09326, 0x24fbb, 0x076c2, 0x00430, 0x23d49, 0x37644, 0x14f3d, 0x13dcf, 0x304b6, 0x1d82b, 0x3e152, 0x393a0,
    0x1aad9, 0x013e3, 0x22a9a, 0x25868, 0x06111, 0x2bd8c, 0x084f5, 0x0f607, 0x2cf7e
};

/**
 * @brief Update the crc with the given buffer
 *
 * @param crc_ptr Pointer to current crc-18 value, gets updated when calculating
 * @param data Buffer to calculate CRC-18 for
 * @param data_len Number of bytes to calculate the crc for
 */
static void crc18_update(uint32_t *crc_ptr, const uint8_t *data, size_t data_len) {
    uint32_t crc = *crc_ptr;
    while (data_len--) {
        crc = (crc << 8) ^ crc18_lookup[((crc >> 10) ^ (*data++)) & 0xff];
    }
    *crc_ptr = crc;
}

// ========================================
// Encoder Functions
// ========================================

void canmore_msg_encode_init(canmore_msg_encoder_t *state, uint32_t client_id, uint32_t direction, bool use_canfd) {
    state->client_id = client_id;
    state->direction = direction;
    state->use_canfd = use_canfd;
    state->length = 0;
    state->position = 0;
}

bool canmore_msg_encode_done(canmore_msg_encoder_t *state) {
    return state->length == state->position;
}

void canmore_msg_encode_load(canmore_msg_encoder_t *state, uint8_t subtype, const uint8_t *buffer, size_t len) {
    if (len > CANMORE_MAX_MSG_LENGTH) {
        len = CANMORE_MAX_MSG_LENGTH;
    }

    state->subtype = subtype;
    state->buf_ptr = buffer;
    state->length = len;
    state->position = 0;
    state->seq_num = 0;
}

bool canmore_msg_encode_next(canmore_msg_encoder_t *state, uint8_t *buffer_out, uint8_t *len_out, uint32_t *id_out,
                             bool *is_extended) {
    if (state->length == state->position) {
        return false;
    }

    size_t max_frame_len = (state->use_canfd ? CANMORE_MAX_FD_FRAME_SIZE : CANMORE_MAX_FRAME_SIZE);

    // Copy buffer
    size_t remaining_size = state->length - state->position;
    size_t copy_size = (max_frame_len > remaining_size ? remaining_size : max_frame_len);
    memcpy(buffer_out, &state->buf_ptr[state->position], copy_size);
    state->position += copy_size;

    // Handle 0 padding for can fd
    if (state->use_canfd) {
        size_t frame_size = canmore_fd_dlc2len(canmore_fd_len2dlc(copy_size));
        if (frame_size != copy_size) {
            memset(&buffer_out[copy_size], 0, frame_size - copy_size);
        }
        *len_out = frame_size;
    }
    else {
        // Normal CAN is simple, all lengths within up to max frame size are valid DLC values
        *len_out = copy_size;
    }

    // Calculate ID
    if (state->seq_num == 0) {
        // First ID, send the first ID format in extended mode
        *is_extended = true;

        bool single = (state->length == state->position);
        *id_out = CANMORE_CALC_MSG_FIRST_ID(state->client_id, state->direction, state->seq_num, single, state->length,
                                            state->subtype);
    }
    else if (state->length == state->position) {
        *is_extended = true;

        uint32_t crc = CRC18_INITIAL_VALUE;
        crc18_update(&crc, state->buf_ptr, state->length);
        crc &= CRC18_MASK;

        *id_out = CANMORE_CALC_MSG_EXT_ID(state->client_id, state->direction, state->seq_num, crc);
    }
    else {
        *is_extended = false;
        *id_out = CANMORE_CALC_MSG_ID(state->client_id, state->direction, state->seq_num);
    }

    // Increment the sequence number
    if (state->seq_num >= CANMORE_MAX_MSG_SEQ_NUM) {
        state->seq_num = 1;
    }
    else {
        state->seq_num++;
    }

    return true;
}

// ========================================
// Decoder Functions
// ========================================

void canmore_msg_decode_reset_state(canmore_msg_decoder_t *state) {
    state->crc18 = CRC18_INITIAL_VALUE;
    state->decode_len = 0;
    state->next_seq_num = 0;
    // Don't reset subtype or data, as these need to be read after this is called
    // No need to reset subtype or expected length, as the decode will always fail until a 0 sequence number is received
    // That code will always write these values from the extended id
}

/**
 * @brief Internal function to report decoding error and resets the decoder state
 *
 * @param state The decoder state to reset
 */
static void decoder_error_and_reset_state(canmore_msg_decoder_t *state, unsigned int error_code) {
    if (state->decode_error_handler) {
        state->decode_error_handler(state->decode_error_arg, error_code);
    }

    canmore_msg_decode_reset_state(state);
}

void canmore_msg_decode_init(canmore_msg_decoder_t *state, canmore_msg_decoder_error_handler_t decode_error_handler,
                             void *decode_error_arg, bool use_canfd) {
    state->decode_error_arg = decode_error_arg;
    state->decode_error_handler = decode_error_handler;
    state->use_canfd = use_canfd;

    canmore_msg_decode_reset_state(state);
}

size_t canmore_msg_decode_frame(canmore_msg_decoder_t *state, uint32_t can_id, bool is_extended, const uint8_t *frame,
                                size_t frame_len) {
    canmore_id_t id;
    id.identifier = can_id;

    uint8_t seq_num = (is_extended ? id.pkt_ext.noc : id.pkt_std.noc);
    bool is_last;
    bool is_single;
    uint32_t crc_expected = UINT32_MAX;

    if (seq_num == 0) {
        // Special handling for first packet, need to store the information in the extended header

        if (state->next_seq_num != 0) {
            // We aren't expecting a new packet, report that we got a bad sequence number
            decoder_error_and_reset_state(state, CANMORE_MSG_DECODER_ERROR_BAD_SEQ_NUM);
            // Don't fail though, since we got a new packet we can just start fresh
        }

        if (!is_extended) {
            decoder_error_and_reset_state(state, CANMORE_MSG_DECODER_ERROR_SEQ_ZERO_NOT_EXTENDED);
            return 0;
        }

        if (id.pkt_ext_start.msg_len > CANMORE_MAX_MSG_LENGTH || id.pkt_ext_start.msg_len == 0) {
            decoder_error_and_reset_state(state, CANMORE_MSG_DECODER_ERROR_MSG_TOO_LARGE);
            return 0;
        }

        state->subtype = id.pkt_ext_start.msg_subtype;
        state->expected_len = id.pkt_ext_start.msg_len;
        is_last = !!id.pkt_ext_start.msg_single;
        is_single = is_last;
    }
    else {
        // Not the first packet, do the standard ID decoding
        is_last = is_extended;
        is_single = false;
        if (is_extended) {
            crc_expected = id.pkt_ext.extra;
        }

        // If not the sequence number we expect, fail now
        if (seq_num != state->next_seq_num) {
            decoder_error_and_reset_state(state, CANMORE_MSG_DECODER_ERROR_BAD_SEQ_NUM);
            return 0;
        }
    }

    // Cannot decode zero length packet
    if (frame_len == 0) {
        decoder_error_and_reset_state(state, CANMORE_MSG_DECODER_ERROR_ZERO_LENGTH_PACKET);
        return 0;
    }

    // Remove any extra data at the end of the last frame (due to CAN FD)
    size_t copy_len = frame_len;
    if (frame_len + state->decode_len > state->expected_len) {
        if (!is_last || !state->use_canfd) {
            decoder_error_and_reset_state(state, CANMORE_MSG_DECODER_ERROR_UNEXPECTED_DATA);
            return 0;
        }

        // If we're the last CAN FD frame, trim to the expected length
        copy_len = state->expected_len - state->decode_len;
    }

    // Compute the checksum (only needed if not single frame packet, since those can't be fragmented)
    if (!is_single) {
        crc18_update(&state->crc18, frame, copy_len);
    }

    // Append packet to end of decode buffer
    memcpy(&state->decode_buffer[state->decode_len], frame, copy_len);
    state->decode_len += copy_len;

    // Finish Processing
    if (is_last) {
        if (state->decode_len < state->expected_len) {
            // If we reached the end of the packet, but we haven't decoded the full expected length, raise an error
            decoder_error_and_reset_state(state, CANMORE_MSG_DECODER_ERROR_MSG_TOO_SMALL);
            return 0;
        }

        // If we're the last packet (and not a single packet transmission), verify the complete message checksum
        if (!is_single) {
            uint32_t crc_calc = state->crc18 & CRC18_MASK;
            if (crc_calc != crc_expected) {
                // Invalid CRC
                decoder_error_and_reset_state(state, CANMORE_MSG_DECODER_ERROR_CRC_FAIL);
                return 0;
            }
        }

        // Reset state for new message
        size_t decoded_len = state->decode_len;
        canmore_msg_decode_reset_state(state);
        return decoded_len;
    }
    else {
        // Make sure if we aren't the last packet that we haven't exceeded the max expected length
        if (state->decode_len >= state->expected_len) {
            decoder_error_and_reset_state(state, CANMORE_MSG_DECODER_ERROR_MSG_TOO_LARGE);
            return 0;
        }

        // If we're not the last packet, increment handling sequence number rollover
        if (state->next_seq_num >= CANMORE_MAX_MSG_SEQ_NUM) {
            if (!state->use_canfd) {
                state->next_seq_num = 1;
            }
            else {
                // Rollover is not permitted in CAN FD mode, since it should fit the full packet in it
                decoder_error_and_reset_state(state, CANMORE_MSG_DECODER_ERROR_ROLLOVER_WITH_CANFD);
                return 0;
            }
        }
        else {
            state->next_seq_num++;
        }

        // We successfully processed, but we don't have a message to give
        return 0;
    }
}

// ========================================
// CAN FD DLC conversion
// ========================================

static const uint8_t dlc2len[] = { 0, 1, 2, 3, 4, 5, 6, 7, 8, 12, 16, 20, 24, 32, 48, 64 };

/* get data length from raw data length code (DLC) */
uint8_t canmore_fd_dlc2len(uint8_t dlc) {
    return dlc2len[dlc & 0x0F];
}

static const uint8_t len2dlc[] = { 0,  1,  2,  3,  4,  5,  6,  7,  8, /* 0 - 8 */
                                   9,  9,  9,  9,                     /* 9 - 12 */
                                   10, 10, 10, 10,                    /* 13 - 16 */
                                   11, 11, 11, 11,                    /* 17 - 20 */
                                   12, 12, 12, 12,                    /* 21 - 24 */
                                   13, 13, 13, 13, 13, 13, 13, 13,    /* 25 - 32 */
                                   14, 14, 14, 14, 14, 14, 14, 14,    /* 33 - 40 */
                                   14, 14, 14, 14, 14, 14, 14, 14,    /* 41 - 48 */
                                   15, 15, 15, 15, 15, 15, 15, 15,    /* 49 - 56 */
                                   15, 15, 15, 15, 15, 15, 15, 15 };  /* 57 - 64 */

/* map the sanitized data length to an appropriate data length code */
uint8_t canmore_fd_len2dlc(uint8_t len) {
    if (len > 64)
        return 0xF;

    return len2dlc[len];
}
