#ifndef CRC32_H
#define CRC32_H

#include <stddef.h>
#include <stdint.h>

/**
 * @brief Compute crc32 of provided buffer
 *
 * @param data Buffer to compute
 * @param len Length of buffer
 * @return uint32_t Resulting crc32
 */
uint32_t crc32_compute(const uint8_t *data, size_t len);

#endif
