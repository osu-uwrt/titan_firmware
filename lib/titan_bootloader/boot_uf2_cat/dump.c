#include "boot/uf2.h"

#include <assert.h>
#include <errno.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>

#define BOOTLOADER_SIZE 0x4000
#define FLASH_BASE 0x10000000
#define FLASH_SIZE (16 * 1024 * 1024)  // 16 MB Flash size
#define UF2_PAGE_SIZE 256              // All RP2040 UF2 files are have 256 bytes of data for flashing

#define FLASH_USAGE_ARRAY_SIZE (FLASH_SIZE / (UF2_PAGE_SIZE * 8))
static_assert(FLASH_SIZE % (UF2_PAGE_SIZE * 8) == 0, "Unaligned flash size");

struct uf2_handle {
    const char *filename;
    FILE *fp;
    uint32_t base_addr;
    uint32_t num_blocks;
};

// #define DEBUG_VERIFY(...) do {} while(0)
#define DEBUG_VERIFY(...) printf(__VA_ARGS__)

bool is_block_valid(struct uf2_block *block, uint32_t expected_num_blocks, bool verify_num_blocks) {
    // Make sure UF2 magics are valid
    if (block->magic_start0 != UF2_MAGIC_START0) {
        DEBUG_VERIFY("Invalid magic start0: 0x%08x\n", block->magic_start0);
        return false;
    }
    if (block->magic_start1 != UF2_MAGIC_START1) {
        DEBUG_VERIFY("Invalid magic start1: 0x%08x\n", block->magic_start1);
        return false;
    }
    if (block->magic_end != UF2_MAGIC_END) {
        DEBUG_VERIFY("Invalid magic end: 0x%08x\n", block->magic_end);
        return false;
    }

    // Ensure that the only flag that is present is the one marking that file_size is family id
    if (block->flags != UF2_FLAG_FAMILY_ID_PRESENT) {
        DEBUG_VERIFY("Unexpected flags: 0x%08x\n", block->flags);
        return false;
    }

    // Make sure family ID matches
    if (block->file_size != RP2040_FAMILY_ID) {
        DEBUG_VERIFY("Invalid family id: 0x%08x\n", block->file_size);
        return false;
    }

    // Ensure page is expected size
    if (block->payload_size != UF2_PAGE_SIZE) {
        DEBUG_VERIFY("Invalid payload size: 0x%08x\n", block->payload_size);
        return false;
    }

    // Check address is page aligned
    if (block->target_addr % UF2_PAGE_SIZE != 0) {
        DEBUG_VERIFY("Invalid address: 0x%08x unaligned\n", block->target_addr);
        return false;
    }

    // Compute valid address range
    // Note max address is the end address minus page size
    uint32_t min_addr = FLASH_BASE;
    uint32_t max_addr = FLASH_BASE + FLASH_SIZE - UF2_PAGE_SIZE;

    // Check address is valid within flash
    if (block->target_addr < min_addr || block->target_addr > max_addr) {
        DEBUG_VERIFY("Invalid address: 0x%08x out of range (0x%08x - 0x%08x)\n", block->target_addr, min_addr,
                     max_addr);
        return false;
    }

    // Check block number makes sense
    if (block->block_no >= block->num_blocks) {
        DEBUG_VERIFY("Invalid block no: 0x%08x >= num blocks 0x%08x\n", block->block_no, block->num_blocks);
        return false;
    }

    // Verify total block count if provided
    if (verify_num_blocks && expected_num_blocks != block->num_blocks) {
        DEBUG_VERIFY("Invalid num blocks: 0x%08x != expected 0x%08x\n", block->num_blocks, expected_num_blocks);
        return false;
    }

    return true;
}

bool open_uf2(const char *filename, struct uf2_handle *handle_out) {
    FILE *f = fopen(filename, "r");
    if (f == NULL) {
        printf("[%s] Failed to open file: %s\n", filename, strerror(errno));
        return false;
    }

    struct uf2_block block;

    size_t readsize = fread(&block, 1, sizeof(block), f);
    if (readsize != sizeof(block)) {
        if (feof(f)) {
            printf("[%s] Unexpected end of UF2 file\n", filename);
        }
        else {
            printf("[%s] Failed to read uf2 block: %s\n", filename, strerror(errno));
        }
        goto fail;
    }

    if (fseek(f, 0, SEEK_SET)) {
        printf("[%s] Failed to rewind: %s\n", filename, strerror(errno));
        goto fail;
    }

    if (!is_block_valid(&block, 0, false)) {
        printf("[%s] Invalid First UF2 Block\n", filename);
        goto fail;
    }

    if (block.block_no != 0) {
        printf("[%s] First UF2 block is not block 0\n", filename);
        goto fail;
    }

    handle_out->fp = f;
    handle_out->filename = filename;
    handle_out->num_blocks = block.num_blocks;
    handle_out->base_addr = block.target_addr;

    return true;

fail:
    fclose(f);
    return false;
}

bool dump_uf2(struct uf2_handle *handle) {
    struct uf2_block block;
    uint32_t expected_block_no = 0;
    uint32_t expected_addr = handle->base_addr;

    while (1) {
        size_t readsize = fread(&block, 1, sizeof(block), handle->fp);
        // Check if at end of file
        if (readsize == 0 && feof(handle->fp)) {
            break;
        }

        if (readsize != sizeof(block)) {
            if (feof(handle->fp)) {
                printf("[%s] Unexpected end of UF2 file\n", handle->filename);
            }
            else {
                printf("[%s] Failed to read uf2 block: %s\n", handle->filename, strerror(errno));
            }
            return false;
        }

        // Make sure uf2 block is valid
        if (!is_block_valid(&block, handle->num_blocks, true)) {
            printf("[%s] Invalid UF2 block found\n", handle->filename);
            return false;
        }

        // Ensure that the block numbering is sequential
        if (expected_block_no != block.block_no) {
            printf("[%s] Out of order UF2 block (%d expected, %d found)\n", handle->filename, expected_block_no,
                   block.block_no);
            return false;
        }
        expected_block_no++;

        // Ensure that target address is in-order and contiguous
        if (expected_addr != block.target_addr) {
            printf("[%s] Non-contiguous UF2 address (0x%08x expected, 0x%08x found)\n", handle->filename, expected_addr,
                   block.target_addr);
            return false;
        }
        expected_addr += UF2_PAGE_SIZE;

        printf("[%s] Block @0x%08x (#%d)\n", handle->filename, block.target_addr, block.block_no);
    }

    if (expected_block_no != handle->num_blocks) {
        printf("[%s] Expected additional UF2 blocks\n", handle->filename);
        return false;
    }

    return true;
}

int main(int argc, char **argv) {
    struct uf2_handle handle;

    if (argc < 2) {
        printf("Usage: %s <uf2 file>\n", argv[0]);
        return 1;
    }

    if (!open_uf2(argv[1], &handle))
        return 1;
    dump_uf2(&handle);

cleanup:
    if (fclose(handle.fp)) {
        printf("[%s] Failed to close file: %s\n", handle.filename, strerror(errno));
        return 1;
    }

    return 0;
}
