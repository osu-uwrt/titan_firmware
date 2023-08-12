#include <algorithm>
#include <cstring>
#include <memory>

#include "boot/uf2.h"

#include "UploadTool.hpp"

using namespace UploadTool;

void assertBlockValid(struct uf2_block *block, uint32_t expected_num_blocks, bool verify_num_blocks) {
    // Make sure UF2 magics are valid
    if (block->magic_start0 != UF2_MAGIC_START0) {
        throw RP2040UF2Error("Invalid UF2 magic start0: " + hexWord(block->magic_start0));
    }
    if (block->magic_start1 != UF2_MAGIC_START1) {
        throw RP2040UF2Error("Invalid UF2 magic start1: " + hexWord(block->magic_start1));
    }
    if (block->magic_end != UF2_MAGIC_END) {
        throw RP2040UF2Error("Invalid UF2 magic end: " + hexWord(block->magic_end));
    }

    // Ensure that the only flag that is present is the one marking that file_size is family id
    if (block->flags != UF2_FLAG_FAMILY_ID_PRESENT) {
        throw RP2040UF2Error("Unexpected flags: " + hexWord(block->flags));
    }

    // Make sure family ID matches
    if (block->file_size != RP2040_FAMILY_ID) {
        throw RP2040UF2Error("Invalid family id: " + hexWord(block->file_size));
    }

    // Ensure page is expected size
    if (block->payload_size != UF2_PAGE_SIZE) {
        throw RP2040UF2Error("Invalid payload size: " + hexWord(block->payload_size));
    }

    // Check address is page aligned
    if (block->target_addr % UF2_PAGE_SIZE != 0) {
        throw RP2040UF2Error("Invalid address: " + hexWord(block->target_addr) + " unaligned");
    }

    // Compute valid address range
    // Note max address is the end address minus page size
    uint32_t min_addr = FLASH_BASE;
    uint32_t max_addr = (FLASH_BASE + MAX_FLASH_SIZE) - UF2_PAGE_SIZE;

    // Check address is valid within flash
    if (block->target_addr < min_addr || block->target_addr > max_addr) {
        throw RP2040UF2Error("Invalid address: " + hexWord(block->target_addr) + " out of range of flash");
    }

    // Check block number makes sense
    if (block->block_no >= block->num_blocks) {
        throw RP2040UF2Error("Invalid block no: " + hexWord(block->block_no) + " >= num blocks " + hexWord(block->num_blocks));
    }

    // Verify total block count if provided
    if (verify_num_blocks && expected_num_blocks != block->num_blocks) {
        throw RP2040UF2Error("Invalid num blocks: " + hexWord(block->num_blocks) + " != expected " + hexWord(expected_num_blocks));
    }
}

RP2040UF2::RP2040UF2(std::ifstream &stream) {
    initFromStream(stream);
}
RP2040UF2::RP2040UF2(const char *filename) {
    std::ifstream stream(filename, std::ifstream::binary);
    initFromStream(stream);
}

void RP2040UF2::initFromStream(std::ifstream &stream) {
    uint32_t expected_block_no = 0;
    uint32_t expected_num_blocks = 0;
    uint32_t expected_addr = 0;
    bool first_block = true;

    do {
        struct uf2_block block;
        stream.read((char*) &block, sizeof(block));

        // First check if the read didn't fully complete
        if (stream.eof()) {
            throw RP2040UF2Error("Unexpected end of UF2 file");
        }

        // Catch any other errors
        if (stream.fail()) {
            throw RP2040UF2Error(std::string("Unexpected IO Error - ") + std::strerror(errno));
        }

        // Perform checks on UF2 block
        assertBlockValid(&block, expected_num_blocks, !first_block);

        // Make sure block numbers are in order
        if (expected_block_no != block.block_no) {
            throw RP2040UF2Error("Out of order UF2 block");
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
            throw RP2040UF2Error("Non-contiguous UF2 address");
        }
        expected_addr += UF2_PAGE_SIZE;

        std::array<uint8_t, UF2_PAGE_SIZE> uf2Data;
        std::copy_n(block.data, UF2_PAGE_SIZE, uf2Data.begin());
        uf2Array.push_back(uf2Data);

        // Quit now if we don't have any data left
        // That means we've read the last block and no data remains after
        stream.peek();
    } while(!stream.eof());

    if (expected_block_no != expected_num_blocks) {
        throw RP2040UF2Error("Expected additional UF2 blocks");
    }

    // Now that we have loaded, fetch all the binary info
    uint32_t searchBase = 0;
    do {
        BinaryInfo::AppInfo app = {};
        app.binaryStart = (searchBase == 0 ? getBaseAddress() : searchBase);
        BinaryInfo::extractAppInfo(*this, app, searchBase);
        searchBase = app.blAppBase;
        if (boardType != "" && app.boardType != "") {
            if (boardType != app.boardType) {
                throw RP2040UF2Error("Mismatched board types");
            }
        }
        else if (boardType == "") {
            boardType = app.boardType;
        }

        apps.push_back(std::move(app));
    } while (searchBase != 0);
}

std::array<uint8_t, 256>& RP2040UF2::getBlock(uint32_t blockNum) {
    return uf2Array.at(blockNum);
}

std::array<uint8_t, 256>& RP2040UF2::getAddress(uint32_t flashAddress) {
    if (flashAddress % UF2_PAGE_SIZE != 0) {
        throw std::logic_error("Unaligned uf2 address read");
    }
    if (flashAddress < baseAddress || flashAddress >= baseAddress + getSize()) {
        throw std::logic_error("Invalid flash address");
    }

    return getBlock((flashAddress - baseAddress) / UF2_PAGE_SIZE);
}
