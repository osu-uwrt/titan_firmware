#include "boot/uf2.h"

#include <assert.h>
#include <errno.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>

#define BOOTLOADER_SIZE 0x4000
#define FLASH_BASE 0x10000000
#define FLASH_SIZE (16 * 1024 * 1024)  // 16 MB Flash size
#define UF2_PAGE_SIZE 256              // All RP2040 UF2 files are have 256 bytes of data for flashing

struct uf2_handle {
    const char *filename;
    FILE *fp;
    uint32_t base_addr;
    uint32_t num_blocks;
    bool is_bootloader;
};

struct uf2_write_handle {
    const char *filename;
    FILE *fp;
    uint32_t num_blocks;   // Total number of blocks to write
    uint32_t block_count;  // The current count of blocks written to fp
    uint32_t next_addr;    // The next address to write in the uf2 (needed for contiguous UF2s), only valid when
                           // block_count > 0
};

// #define DEBUG_VERIFY(...) do {} while(0)
#define DEBUG_VERIFY(...) printf(__VA_ARGS__)

bool is_block_valid(struct uf2_block *block, bool is_bootloader, uint32_t expected_num_blocks, bool verify_num_blocks) {
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
    uint32_t min_addr = (is_bootloader ? FLASH_BASE : FLASH_BASE + BOOTLOADER_SIZE);
    uint32_t max_addr = (is_bootloader ? FLASH_BASE + BOOTLOADER_SIZE : FLASH_BASE + FLASH_SIZE) - UF2_PAGE_SIZE;

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

bool open_uf2(const char *filename, bool is_bootloader, struct uf2_handle *handle_out) {
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

    if (!is_block_valid(&block, is_bootloader, 0, false)) {
        printf("[%s] Invalid First UF2 Block\n", filename);
        goto fail;
    }

    if (block.block_no != 0) {
        printf("[%s] First UF2 block is not block 0\n", filename);
        goto fail;
    }

    handle_out->fp = f;
    handle_out->filename = filename;
    handle_out->is_bootloader = is_bootloader;
    handle_out->num_blocks = block.num_blocks;
    handle_out->base_addr = block.target_addr;

    return true;

fail:
    fclose(f);
    return false;
}

bool create_uf2(const char *filename, struct uf2_write_handle *handle_out) {
    FILE *f = fopen(filename, "w");
    if (f == NULL) {
        printf("[%s] Failed to open output file: %s\n", filename, strerror(errno));
        return false;
    }

    handle_out->filename = filename;
    handle_out->fp = f;
    handle_out->block_count = 0;
    handle_out->num_blocks = 0;
    handle_out->next_addr = 0;

    return true;
}

static struct uf2_block padding_block = {
    .magic_start0 = UF2_MAGIC_START0,
    .magic_start1 = UF2_MAGIC_START1,
    .flags = UF2_FLAG_FAMILY_ID_PRESENT,
    .payload_size = UF2_PAGE_SIZE,
    .file_size = RP2040_FAMILY_ID,
    .magic_end = UF2_MAGIC_END,
};

uint32_t calc_required_padding_blocks(uint32_t prev_base, uint32_t prev_block_count, uint32_t next_base) {
    uint32_t prev_end = prev_base + (prev_block_count * UF2_PAGE_SIZE);

    // This should be caught by the caller, but just check to be sure before the math breaks down
    assert(prev_end <= next_base);

    uint32_t padding_bytes = next_base - prev_end;

    // Ensure that the padding is aligned
    // Again, this should be caught by the caller, but just to be safe
    assert(padding_bytes % UF2_PAGE_SIZE == 0);

    return padding_bytes / UF2_PAGE_SIZE;
}

bool pad_uf2(struct uf2_write_handle *write_handle, uint32_t target_addr) {
    if (write_handle->block_count == 0) {
        write_handle->next_addr = target_addr;
        return true;
    }

    if (target_addr < write_handle->next_addr) {
        DEBUG_VERIFY("Cannot pad to address 0x%08x < 0x%08x\n", target_addr, write_handle->next_addr);
        return false;
    }

    if (target_addr % 256 != 0) {
        DEBUG_VERIFY("Cannot pad to unaligned address 0x%08x\n", target_addr);
        return false;
    }

    padding_block.num_blocks = write_handle->num_blocks;

    while (write_handle->next_addr < target_addr) {
        padding_block.block_no = write_handle->block_count++;
        padding_block.target_addr = write_handle->next_addr;
        write_handle->next_addr += UF2_PAGE_SIZE;

        size_t written = fwrite(&padding_block, sizeof(padding_block), 1, write_handle->fp);
        if (written != 1) {
            printf("[%s] Failed to write output file: %s", write_handle->filename, strerror(errno));
            return false;
        }
    }

    // If this isn't true, the program logic is messed up
    assert(write_handle->next_addr == target_addr);

    return true;
}

bool append_uf2(struct uf2_handle *handle, struct uf2_write_handle *write_handle) {
    struct uf2_block block;
    uint32_t expected_block_no = 0;

    // First insert the required padding for the uf2 file
    if (!pad_uf2(write_handle, handle->base_addr)) {
        printf("[%s] Failed to pad output file before writing contents\n", handle->filename);
        return false;
    }

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
        if (!is_block_valid(&block, handle->is_bootloader, handle->num_blocks, true)) {
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
        if (write_handle->next_addr != block.target_addr) {
            printf("[%s] Non-contiguous UF2 address (0x%08x expected, 0x%08x found)\n", handle->filename,
                   write_handle->next_addr, block.target_addr);
            return false;
        }
        write_handle->next_addr += UF2_PAGE_SIZE;

        // Block appears to check out, now the block counts need to be fixed up
        block.block_no = write_handle->block_count++;
        block.num_blocks = write_handle->num_blocks;

        size_t written = fwrite(&block, sizeof(block), 1, write_handle->fp);
        if (written != 1) {
            printf("[%s] Failed to write output file: %s", write_handle->filename, strerror(errno));
            return false;
        }
    }

    if (expected_block_no != handle->num_blocks) {
        printf("[%s] Expected additional UF2 blocks\n", handle->filename);
        return false;
    }

    return true;
}

int main(int argc, char **argv) {
    struct uf2_write_handle write_handle;
    struct uf2_handle bl_handle, app_handle;
    bool successful = false;

    // Initialize padding block properly
    memset(padding_block.data, 0xFF, UF2_PAGE_SIZE);

    if (argc < 4) {
        printf("Usage: %s <bootloader uf2> <app uf2> <output uf2>\n", argv[0]);
        return 1;
    }

    // Create output file
    if (!create_uf2(argv[3], &write_handle)) {
        return 1;
    }

    // Open input files
    if (!open_uf2(argv[1], true, &bl_handle))
        goto cleanup_output;
    write_handle.num_blocks += bl_handle.num_blocks;
    if (!open_uf2(argv[2], false, &app_handle))
        goto cleanup_bl;
    write_handle.num_blocks +=
        calc_required_padding_blocks(bl_handle.base_addr, bl_handle.num_blocks, app_handle.base_addr);
    write_handle.num_blocks += app_handle.num_blocks;

    // Append output files
    if (!append_uf2(&bl_handle, &write_handle))
        goto cleanup;
    if (!append_uf2(&app_handle, &write_handle))
        goto cleanup;

    // In theory other checks should catch this, but just make sure all blocks have been written
    assert(write_handle.block_count == write_handle.num_blocks);
    successful = true;

cleanup:
    if (fclose(app_handle.fp)) {
        printf("[%s] Failed to close file: %s\n", write_handle.filename, strerror(errno));
        successful = false;
    }

cleanup_bl:
    if (fclose(bl_handle.fp)) {
        printf("[%s] Failed to close file: %s\n", write_handle.filename, strerror(errno));
        successful = false;
    }

cleanup_output:
    if (fclose(write_handle.fp)) {
        printf("[%s] Failed to close output file: %s\n", write_handle.filename, strerror(errno));
        successful = false;
    }

    if (!successful) {
        if (remove(write_handle.filename)) {
            printf("[%s] Failed to delete incomplete output file: %s\n", write_handle.filename, strerror(errno));
        }
    }

    return (successful ? 0 : 1);
}
