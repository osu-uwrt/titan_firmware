#include <string.h>

#include "canmore_msg_encoding.h"

// ========================================
// CRC-18 Calculations
// ========================================

#define CRC18_INITIAL_VALUE 0x3FFFF
#define CRC18_MASK 0x3FFFF

static const uint32_t crc18_lookup[] = {
    0x00000, 0x23979, 0x24b8b, 0x072f2, 0x2ae6f, 0x09716, 0x0e5e4, 0x2dc9d,
    0x365a7, 0x15cde, 0x12e2c, 0x31755, 0x1cbc8, 0x3f2b1, 0x38043, 0x1b93a,
    0x0f237, 0x2cb4e, 0x2b9bc, 0x080c5, 0x25c58, 0x06521, 0x017d3, 0x22eaa,
    0x39790, 0x1aee9, 0x1dc1b, 0x3e562, 0x139ff, 0x30086, 0x37274, 0x14b0d,
    0x1e46e, 0x3dd17, 0x3afe5, 0x1969c, 0x34a01, 0x17378, 0x1018a, 0x338f3,
    0x281c9, 0x0b8b0, 0x0ca42, 0x2f33b, 0x02fa6, 0x216df, 0x2642d, 0x05d54,
    0x11659, 0x32f20, 0x35dd2, 0x164ab, 0x3b836, 0x1814f, 0x1f3bd, 0x3cac4,
    0x273fe, 0x04a87, 0x03875, 0x2010c, 0x0dd91, 0x2e4e8, 0x2961a, 0x0af63,
    0x3c8dc, 0x1f1a5, 0x18357, 0x3ba2e, 0x166b3, 0x35fca, 0x32d38, 0x11441,
    0x0ad7b, 0x29402, 0x2e6f0, 0x0df89, 0x20314, 0x03a6d, 0x0489f, 0x271e6,
    0x33aeb, 0x10392, 0x17160, 0x34819, 0x19484, 0x3adfd, 0x3df0f, 0x1e676,
    0x05f4c, 0x26635, 0x214c7, 0x02dbe, 0x2f123, 0x0c85a, 0x0baa8, 0x283d1,
    0x22cb2, 0x015cb, 0x06739, 0x25e40, 0x082dd, 0x2bba4, 0x2c956, 0x0f02f,
    0x14915, 0x3706c, 0x3029e, 0x13be7, 0x3e77a, 0x1de03, 0x1acf1, 0x39588,
    0x2de85, 0x0e7fc, 0x0950e, 0x2ac77, 0x070ea, 0x24993, 0x23b61, 0x00218,
    0x1bb22, 0x3825b, 0x3f0a9, 0x1c9d0, 0x3154d, 0x12c34, 0x15ec6, 0x367bf,
    0x1a8c1, 0x391b8, 0x3e34a, 0x1da33, 0x306ae, 0x13fd7, 0x14d25, 0x3745c,
    0x2cd66, 0x0f41f, 0x086ed, 0x2bf94, 0x06309, 0x25a70, 0x22882, 0x011fb,
    0x15af6, 0x3638f, 0x3117d, 0x12804, 0x3f499, 0x1cde0, 0x1bf12, 0x3866b,
    0x23f51, 0x00628, 0x074da, 0x24da3, 0x0913e, 0x2a847, 0x2dab5, 0x0e3cc,
    0x04caf, 0x275d6, 0x20724, 0x03e5d, 0x2e2c0, 0x0dbb9, 0x0a94b, 0x29032,
    0x32908, 0x11071, 0x16283, 0x35bfa, 0x18767, 0x3be1e, 0x3ccec, 0x1f595,
    0x0be98, 0x287e1, 0x2f513, 0x0cc6a, 0x210f7, 0x0298e, 0x05b7c, 0x26205,
    0x3db3f, 0x1e246, 0x190b4, 0x3a9cd, 0x17550, 0x34c29, 0x33edb, 0x107a2,
    0x2601d, 0x05964, 0x02b96, 0x212ef, 0x0ce72, 0x2f70b, 0x285f9, 0x0bc80,
    0x105ba, 0x33cc3, 0x34e31, 0x17748, 0x3abd5, 0x192ac, 0x1e05e, 0x3d927,
    0x2922a, 0x0ab53, 0x0d9a1, 0x2e0d8, 0x03c45, 0x2053c, 0x277ce, 0x04eb7,
    0x1f78d, 0x3cef4, 0x3bc06, 0x1857f, 0x359e2, 0x1609b, 0x11269, 0x32b10,
    0x38473, 0x1bd0a, 0x1cff8, 0x3f681, 0x12a1c, 0x31365, 0x36197, 0x158ee,
    0x0e1d4, 0x2d8ad, 0x2aa5f, 0x09326, 0x24fbb, 0x076c2, 0x00430, 0x23d49,
    0x37644, 0x14f3d, 0x13dcf, 0x304b6, 0x1d82b, 0x3e152, 0x393a0, 0x1aad9,
    0x013e3, 0x22a9a, 0x25868, 0x06111, 0x2bd8c, 0x084f5, 0x0f607, 0x2cf7e
};

/**
 * @brief Update the crc with the given buffer
 *
 * @param crc_ptr Pointer to current crc-18 value, gets updated when calculating
 * @param data Buffer to calculate CRC-18 for
 * @param data_len Number of bytes to calculate the crc for
*/
static void crc18_update(uint32_t *crc_ptr, uint8_t *data, size_t data_len)
{
    uint32_t crc = *crc_ptr;
    while (data_len--) {
        crc = (crc << 8) ^ crc18_lookup[((crc >> 10) ^ (*data++)) & 0xff];
    }
    *crc_ptr = crc;
}

// ========================================
// Encoder Functions
// ========================================

void canmore_msg_encode_init(canmore_msg_encoder_t *state, uint32_t client_id, uint32_t direction) {
    state->client_id = client_id;
    state->direction = direction;
    state->length = 0;
    state->position = 0;
}

bool canmore_msg_encode_done(canmore_msg_encoder_t *state) {
    return state->length == state->position;
}

void canmore_msg_encode_load(canmore_msg_encoder_t *state, uint8_t *buffer, size_t len) {
    if (len > CANMORE_MAX_MSG_LENGTH) {
        len = CANMORE_MAX_MSG_LENGTH;
    }

    memcpy(state->buffer, buffer, len);
    state->length = len;
    state->position = 0;
    state->seq_num = 0;
}

bool canmore_msg_encode_next(canmore_msg_encoder_t *state, uint8_t *buffer_out, uint32_t *dlc_out, uint32_t *id_out, bool *is_extended) {
    if (state->length == state->position) {
        return false;
    }

    // Copy buffer
    size_t remaining_size = state->length - state->position;
    size_t copy_size = (CANMORE_FRAME_SIZE > remaining_size ? remaining_size : CANMORE_FRAME_SIZE);
    memcpy(buffer_out, &state->buffer[state->position], copy_size);
    *dlc_out = copy_size;
    state->position += copy_size;

    // Calculate ID
    if (state->length == state->position) {
        *is_extended = true;

        uint32_t crc = CRC18_INITIAL_VALUE;
        crc18_update(&crc, state->buffer, state->length);
        crc &= CRC18_MASK;

        *id_out = CANMORE_CALC_MSG_EXT_ID(state->client_id, state->direction, state->seq_num, crc);
    } else {
        *is_extended = false;
        *id_out = CANMORE_CALC_MSG_ID(state->client_id, state->direction, state->seq_num);
    }
    state->seq_num++;

    return true;
}


// ========================================
// Decoder Functions
// ========================================

void canmore_msg_decode_reset_state(canmore_msg_decoder_t *state) {
    state->crc18 = CRC18_INITIAL_VALUE;
    state->decode_len = 0;
    state->next_seq_num = 0;
}

/**
 * @brief Internal function to report decoding error and resets the decoder state
 *
 * @param state The decoder state to reset
*/
static void decoder_error_and_reset_state(canmore_msg_decoder_t *state) {
    // TODO: Report error, but only if the last received frame also wasn't an error (need a successful read before printing again)
    canmore_msg_decode_reset_state(state);
}

void canmore_msg_decode_init(canmore_msg_decoder_t *state) {
    canmore_msg_decode_reset_state(state);
}

bool canmore_msg_decode_frame(canmore_msg_decoder_t *state, uint8_t seq_num, uint8_t *data, size_t data_len) {
    // Ensure next frame sequence matches expected order
    if (seq_num != state->next_seq_num) {
        decoder_error_and_reset_state(state);

        if (seq_num != 0) {
            return false;
        }
        // If seq_num is 0, then it would be reset when calling this anyways, so it can continue processing the frame
    }

    // Cannot decode zero length packet
    if (data_len == 0) {
        decoder_error_and_reset_state(state);
        return false;
    }

    // Ensure that buffer overflow will not occur (it shouldn't, but just in case)
    if (data_len + state->decode_len > CANMORE_MAX_MSG_LENGTH) {
        decoder_error_and_reset_state(state);
        return false;
    }

    // Append packet to end of decode buffer
    crc18_update(&state->crc18, data, data_len);
    memcpy(&state->decode_buffer[state->decode_len], data, data_len);
    state->decode_len += data_len;
    state->next_seq_num++;

    return true;
}

size_t canmore_msg_decode_last_frame(canmore_msg_decoder_t *state, uint8_t seq_num, uint8_t *data, size_t data_len, uint32_t crc, uint8_t *data_out) {
    // Process packet data
    if (!canmore_msg_decode_frame(state, seq_num, data, data_len)) {
        return 0;
    }

    // Check CRC
    uint32_t crc_calc = state->crc18 & CRC18_MASK;
    if (crc_calc != crc) {
        // Invalid CRC
        decoder_error_and_reset_state(state);
        return 0;
    }

    // Copy out message
    size_t decode_len = state->decode_len;
    memcpy(data_out, state->decode_buffer, state->decode_len);

    // Reset state for new message
    canmore_msg_decode_reset_state(state);

    return decode_len;
}