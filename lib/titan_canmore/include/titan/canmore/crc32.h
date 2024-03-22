#ifndef TITAN__CANMORE__CRC32_H_
#define TITAN__CANMORE__CRC32_H_

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Updates the original_crc32 with the contents of the provided buffer
 *
 * @param data Buffer to update CRC32 with
 * @param len Length of buffer
 * @param original_crc32 The start CRC32 to update
 * @return uint32_t The updated CRC32
 */
uint32_t crc32_update(const uint8_t *data, size_t len, uint32_t original_crc32);

/**
 * @brief Compute crc32 of provided buffer
 *
 * @param data Buffer to compute
 * @param len Length of buffer
 * @return uint32_t Resulting crc32
 */
uint32_t crc32_compute(const uint8_t *data, size_t len);

#ifdef __cplusplus
}
#endif

#endif  // CRC32_H
