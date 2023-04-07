#include <algorithm>
#include <memory>

#include "boot/uf2.h"

#include "RP2040FlashInterface.hpp"

using namespace UploadTool;

// #define DEBUG_VERIFY(...) do {} while(0)
#define DEBUG_VERIFY(...) printf(__VA_ARGS__)

bool is_block_valid(struct uf2_block *block, bool is_ota, uint32_t expected_num_blocks, bool verify_num_blocks) {
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
    uint32_t min_addr = (is_ota ? FLASH_BASE + BOOTLOADER_SIZE : FLASH_BASE);
    uint32_t max_addr = (FLASH_BASE + MAX_FLASH_SIZE) - UF2_PAGE_SIZE;

    // Check address is valid within flash
    if (block->target_addr < min_addr || block->target_addr > max_addr) {
        DEBUG_VERIFY("Invalid address: 0x%08x out of range (0x%08x - 0x%08x)\n", block->target_addr, min_addr, max_addr);
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

RP2040UF2::RP2040UF2(std::ifstream &stream, bool isOTA): isOTA(isOTA) {
    initFromStream(stream, isOTA);
}
RP2040UF2::RP2040UF2(const char *filename, bool isOTA): isOTA(isOTA) {
    std::ifstream stream(filename, std::ifstream::binary);
    initFromStream(stream, isOTA);
}

void RP2040UF2::initFromStream(std::ifstream &stream, bool isOTA) {
    // Raise exceptions if it tries to incomplete read
    stream.exceptions(stream.exceptions() | std::ios::failbit | std::ifstream::badbit);

    uint32_t expected_block_no = 0;
    uint32_t expected_num_blocks = 0;
    uint32_t expected_addr = 0;
    bool first_block = true;

    do {
        struct uf2_block block;
        stream.read((char*) &block, sizeof(block));

        if (!is_block_valid(&block, isOTA, expected_num_blocks, !first_block)) {
            throw std::runtime_error("Invalid UF2 block");
        }

        // Make sure block numbers are in order
        if (expected_block_no != block.block_no) {
            throw std::runtime_error("Out of order UF2 block");
        }
        expected_block_no++;

        if (first_block) {
            baseAddress = block.target_addr;
            expected_addr = block.target_addr;
            expected_num_blocks = block.num_blocks;
            first_block = false;
        }

        // Ensure that target address is in-order and contiguous
        if (expected_addr != block.target_addr) {
            throw std::runtime_error("Non-contiguous UF2 address");
        }
        expected_addr += UF2_PAGE_SIZE;

        std::array<uint8_t, UF2_PAGE_SIZE> uf2Data;
        std::copy_n(block.data, UF2_PAGE_SIZE, uf2Data.begin());
        uf2Array.push_back(uf2Data);

        stream.peek();
    } while(!stream.eof());

    if (expected_block_no != expected_num_blocks) {
        throw std::runtime_error("Expected additional UF2 blocks");
    }
}

std::array<uint8_t, 256>& RP2040UF2::getBlock(uint32_t blockNum) {
    return uf2Array.at(blockNum);
}

std::array<uint8_t, 256>& RP2040UF2::getAddress(uint32_t flashAddress) {
    if (flashAddress % UF2_PAGE_SIZE != 0) {
        throw std::runtime_error("Unaligned uf2 address read");
    }
    if (flashAddress < baseAddress || flashAddress >= baseAddress + getSize()) {
        throw std::runtime_error("Invalid flash address");
    }

    return getBlock((flashAddress - baseAddress) / UF2_PAGE_SIZE);
}
