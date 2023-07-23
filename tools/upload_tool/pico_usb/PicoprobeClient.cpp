#include <iostream>
#include <chrono>
#include "PicoprobeClient.hpp"

#include "hardware/regs/addressmap.h"
#include "hardware/regs/ssi.h"
#include "hardware/regs/io_qspi.h"

using namespace PicoUSB;

PicoprobeClient::PicoprobeClient(std::string serialStr): target(serialStr), flashState(FlashUnknown) {
    cachedFlashId = readFlashId();

    // Extract current BinaryInfo on device
    try {
        UploadTool::extractBinaryInfo(*this, firstApp);
        if (firstApp.isBootloader) {
            UploadTool::extractBinaryInfo(*this, nestedApp, firstApp.blAppBase);
        }
    }
    catch (std::exception &e) {}    // Ignore any failures during discovery
    catch (...) {}
}

void PicoprobeClient::reboot() {
    flushWriteBuffer();

    // This ensures flash isn't in an unstable state before resetting, as this only resets the core, rather than the whole device
    enterXIPMode();
    target.reboot();
}

void PicoprobeClient::writeBytes(uint32_t addr, std::array<uint8_t, UF2_PAGE_SIZE> &bytes) {
    if (addr < FLASH_BASE || addr >= FLASH_BASE + getFlashSize() || (addr & (UF2_PAGE_SIZE-1)) != 0) {
        throw std::logic_error("Invalid Write Address");
    }

    if (writeBuffer.size() > 0 && writeBufferAddr + (writeBuffer.size()*4) != addr) {
        flushWriteBuffer();
    }
    if (writeBuffer.size() == 0) {
        writeBufferAddr = addr;
    }

    // Convert to vector (required by openocd wrapper) and convert to 32-bit words to increase write speed
    writeBuffer.reserve(writeBuffer.size() + UF2_PAGE_SIZE / 4);
    for (size_t i = 0; i < UF2_PAGE_SIZE; i += 4) {
        uint32_t word = 0;
        word |= bytes.at(i);
        word |= bytes.at(i + 1) << 8;
        word |= bytes.at(i + 2) << 16;
        word |= bytes.at(i + 3) << 24;
        writeBuffer.push_back(word);
    }

    static_assert(SRAM5_BASE - SRAM4_BASE == 4096, "Invalid memory assumptions");
    if (writeBuffer.size() == 4096) {
        flushWriteBuffer();
    }
}

void PicoprobeClient::flushWriteBuffer() {
    if (writeBuffer.size() == 0) return;

    enterCommandMode();

    // Write to last 256 bytes of SCRATCH_Y (where the bootrom normally copies out boot2)
    // Should be a safe place to write, as the bootrom will overwrite it anyways during the boot sequence
    uint32_t scratch_addr = SRAM4_BASE;
    target.writeMemory(scratch_addr, writeBuffer, 32);

    // void flash_range_program(uint32_t addr, const uint8_t *data, size_t count)
    // Program data to a range of flash addresses starting at addr (offset from the start of flash) and count bytes
    // in size. addr must be aligned to a 256-byte boundary, and count must be a multiple of 256.
    target.callFunction(target.lookupRomFunc('R', 'P'), writeBufferAddr - FLASH_BASE, scratch_addr, writeBuffer.size()*4);

    notifyCommandDone();
    writeBuffer.clear();
}


void PicoprobeClient::eraseSector(uint32_t addr) {
    if (addr < FLASH_BASE || addr >= FLASH_BASE + getFlashSize() || (addr & (FLASH_ERASE_SIZE-1)) != 0) {
        throw std::logic_error("Invalid Erase Address");
    }

    flushWriteBuffer();
    enterCommandMode();

    // void flash_range_erase(uint32_t addr, size_t count, uint32_t block_size, uint8_t block_cmd)
    // From Bootrom Source:
    // Erase a count bytes, starting at addr (offset from start of flash).
    // addr must be aligned to a 4096-byte sector, and count must be a multiple of 4096 bytes.
    // block_size must be a power of 2.
    // Generally block_size > 4k, and block_cmd is some command which erases a block
    // of this size. This accelerates erase speed.
    // To use sector-erase only, set block_size to some value larger than flash,
    // e.g. 1ul << 31.
    // To override the default 20h erase cmd, set block_size == 4k.

    // We're going to erase with the default 0x20 erase cmd, and don't want to provide another faster command
    target.callFunction(target.lookupRomFunc('R', 'E'), addr - FLASH_BASE, FLASH_ERASE_SIZE, 1ul << 31, 0);

    notifyCommandDone();
}

void PicoprobeClient::readBytes(uint32_t addr, std::array<uint8_t, UF2_PAGE_SIZE> &bytesOut) {
    if (addr < FLASH_BASE || addr >= FLASH_BASE + getFlashSize() || (addr & (UF2_PAGE_SIZE-1)) != 0) {
        throw std::logic_error("Invalid Read Address");
    }

    flushWriteBuffer();
    enterXIPMode();

    auto result = target.readMemory(addr, bytesOut.size() / 4, 32);
    for (size_t i = 0; i < bytesOut.size(); i+=4) {
        uint32_t word = result.at(i / 4);
        bytesOut.at(i) = word & 0xFF;
        bytesOut.at(i + 1) = (word >> 8) & 0xFF;
        bytesOut.at(i + 2) = (word >> 16) & 0xFF;
        bytesOut.at(i + 3) = (word >> 24) & 0xFF;
    }
}

bool PicoprobeClient::verifyBytes(uint32_t addr, std::array<uint8_t, UF2_PAGE_SIZE> &bytes) {
    std::array<uint8_t, UF2_PAGE_SIZE> readout;
    readBytes(addr, readout);
    return bytes == readout;
}

uint64_t PicoprobeClient::readFlashId() {
    enterCommandMode();

    // Perform Read Unique ID Command
    uint64_t flashId = 0;
    flashCsForce(false);
    ssiTransfer(0x4B);
    for (int i = 0; i < 4; i++) ssiTransfer(0);
    for (int i = 0; i < 8; i++) flashId |= ((uint64_t)ssiTransfer(0) << (8*i));
    flashCsForce(true);

    // TODO: Inline this into assembly
    notifyCommandDone();

    return flashId;
}

void PicoprobeClient::enterXIPMode() {
    if (flashState != FlashXIP) {
        if (flashState != FlashCommand) {
            // If we aren't in command mode, re-run connect internal flash to reset the QSPI pads to known good state
            target.callFunction(target.lookupRomFunc('I', 'F'));
        }
        flashState = FlashUnknown;

        // Call flush cache to reset any manual overrides and clear cache from any commands that were sent
        target.callFunction(target.lookupRomFunc('F', 'C'));
        // Enter XIP mode to allow direct reads to flash
        target.callFunction(target.lookupRomFunc('C', 'X'));
        flashState = FlashXIP;
    }
}

void PicoprobeClient::enterCommandMode() {
    // Set to unknown state until we're done with the command (to prevent exceptions from breaking our state)
    flashState = FlashUnknown;
    auto lastState = flashState;

    // Switch to the required state to begin executing commands
    if (lastState != FlashCommand) {
        // Call _connect_internal_flash() to restore pin muxing
        target.callFunction(target.lookupRomFunc('I', 'F'));
        target.callFunction(target.lookupRomFunc('E', 'X'));
    }
}

void PicoprobeClient::notifyCommandDone() {
    // Mark that a command is done, so we know we're in a known good state
    flashState = FlashCommand;
}

void PicoprobeClient::flashCsForce(bool high) {
    uint32_t ss_ctrl_io_addr = (IO_QSPI_BASE + IO_QSPI_GPIO_QSPI_SS_CTRL_OFFSET);
    uint32_t field_val = high ?
        IO_QSPI_GPIO_QSPI_SS_CTRL_OUTOVER_VALUE_HIGH :
        IO_QSPI_GPIO_QSPI_SS_CTRL_OUTOVER_VALUE_LOW;
    uint32_t new_val = (target.readWord(ss_ctrl_io_addr) & ~(IO_QSPI_GPIO_QSPI_SS_CTRL_OUTOVER_BITS)) | (field_val << IO_QSPI_GPIO_QSPI_SS_CTRL_OUTOVER_LSB);
    target.writeWord(ss_ctrl_io_addr, new_val);
}

uint32_t PicoprobeClient::ssiTransfer(uint32_t tx, int timeout_ms) {
    uint32_t ssi_hw_sr_io_addr = (XIP_SSI_BASE + SSI_SR_OFFSET);
    uint32_t ssi_hw_dr0_io_addr = (XIP_SSI_BASE + SSI_DR0_OFFSET);

    // Check TX fifo not full
    uint32_t sr = 0;
    std::chrono::time_point start = std::chrono::steady_clock::now();
    while (!(sr & SSI_SR_TFNF_BITS)) {
        sr = target.readWord(ssi_hw_sr_io_addr);

        if(std::chrono::steady_clock::now() - start > std::chrono::milliseconds(timeout_ms))
            throw PicoprobeError("Timed out waiting for SSI ready");
    }

    // Write tx fifo
    target.writeWord(ssi_hw_dr0_io_addr, tx);

    // Check rx fifo not empty
    sr = 0;
    start = std::chrono::steady_clock::now();
    while (!(sr & SSI_SR_RFNE_BITS)) {
        sr = target.readWord(ssi_hw_sr_io_addr);

        if(std::chrono::steady_clock::now() - start > std::chrono::milliseconds(timeout_ms))
            throw PicoprobeError("Timed out waiting for SSI to respond");
    }

    // Read RX fifo
    return target.readWord(ssi_hw_dr0_io_addr);
}
