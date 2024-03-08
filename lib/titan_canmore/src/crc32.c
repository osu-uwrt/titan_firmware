#include "canmore/crc32.h"

uint32_t crc32_compute(const uint8_t *data, size_t len) {
    uint32_t crc32 = 0xFFFFFFFF;

    while (len--) {
        crc32 = crc32 ^ ((*data++) << 24);
        for (int i = 0; i < 8; i++) {
            if (crc32 & (1L << 31))
                crc32 = (crc32 << 1) ^ CRC32_POLYNOMIAL;
            else
                crc32 = (crc32 << 1);
        }
    }

    return crc32;
}

int file_crc32_compute(const char *filename, uint32_t *crc32_result) {
    uint32_t crc32 = 0xFFFFFFFF;

    int fd = open(filename, O_RDONLY);
    if (fd < 0) {
        return -1;
    }

    char data;
    int ret;

    // loop until error (-1) or end of file (0) reached
    while (ret = read(fd, &data, 1) > 0) {
        crc32 = crc32 ^ (data << 24);
        for (int i = 0; i < 8; i++) {
            if (crc32 & (1L << 31))
                crc32 = (crc32 << 1) ^ CRC32_POLYNOMIAL;
            else
                crc32 = (crc32 << 1);
        }
    }

    if (close(fd) < 0) {
        return -1;
    }

    // while was broken because the last read from the file resulted in an error
    if (ret < 0) {
        return -1;
    }

    *crc32_result = crc32;
    return 0;
}
