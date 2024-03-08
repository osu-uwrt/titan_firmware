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

/**
 * @brief Compute crc32 of provided buffer
 *
 * @param data Buffer to compute
 * @param len Length of buffer
 * @return uint32_t Resulting crc32
 */
uint32_t crc32_compute(const uint8_t *data, size_t len);

/**
 * @brief Compute crc32 of the file with the path provided to the function
 *
 * @param filename Name of file of which to compute crc32
 * @param crc32 pointer to a 32-bit unsigned int which will be populated with the crc32
 * @return uint32_t 0 if the crc32 was computed successfully, -1 otherwise. The computation may fail if the
 * function encounters file io errors. If this happens, then errno will be set appropriately
 */
int file_crc32_compute(const char *filename, uint32_t *crc32);

#ifdef __cplusplus
}
#endif

#endif  // CRC32_H
