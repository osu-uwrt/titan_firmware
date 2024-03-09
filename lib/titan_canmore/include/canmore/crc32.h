#ifndef CRC32_H
#define CRC32_H

#include <fcntl.h>
#include <stddef.h>
#include <stdint.h>
#include <unistd.h>

#define CRC32_POLYNOMIAL 0x04C11DB7

#ifdef __cplusplus
extern "C" {
#endif

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
