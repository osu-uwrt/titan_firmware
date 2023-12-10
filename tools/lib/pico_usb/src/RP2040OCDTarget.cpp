#include "pico_usb/PicoprobeClient.hpp"

#include "hardware/regs/addressmap.h"
#include "hardware/regs/pads_bank0.h"
#include "hardware/regs/resets.h"

#include <iostream>
#include <sstream>

using namespace PicoUSB;

bool RP2040OCDTarget::showedVersionWarning = false;

RP2040OCDTarget::RP2040OCDTarget(std::string probeSerial): probeSerial(probeSerial) {
    // First initialize openocd in rescue mode to fully reset chip into known good state
    initRescue();

    // Restart openocd since we can't have both the rescue and CPU DAPs in the same instance
    openocd.reset();

    // Perform RP2040 debug initialization
    initNormal();
    readoutRom();
}

void RP2040OCDTarget::initCommon() {
    openocd = std::make_unique<OpenOCDInstance>();

    // Warn about invalid versions
    auto version = openocd->getVersion();

    if (showedVersionWarning) {
    }
    else if (!version.valid) {
        std::cerr
            << "[WARNING] Unable to parse OpenOCD version. This program has been tested to work with version v0.12"
            << std::endl;
        showedVersionWarning = true;
    }
    else if (version.major > 0 || version.minor > 12) {
        std::cerr << "[WARNING] OpenOCD install (detected " << version.str()
                  << ") is newer than tested versions v0.12. This wrapper may not function correctly." << std::endl;
        showedVersionWarning = true;
    }
    else if (version.minor < 12) {
        std::cerr << "[WARNING] OpenOCD install (detected " << version.str()
                  << ") is outdated! This wrapper may not function correctly. Please use v0.12." << std::endl;
        showedVersionWarning = true;
    }

    // Configure probe for openocd
    // Taken from OpenOCD interface/cmsis-dap.cfg
    if (!openocd->ranCustomProbeInit) {
        // Configure picoprobe if a custom probe init wasn't requested
        openocd->sendCommand("adapter driver cmsis-dap");
        if (probeSerial.size() > 0)
            openocd->sendCommand("adapter serial " + probeSerial);
        openocd->sendCommand("adapter speed 5000");
    }

    // Enable SWD
    // Taken from OpenOCD target/rp2040.cfg
    openocd->sendCommand("transport select swd");
    openocd->sendCommand("eval swd newdap rp2040 cpu -expected-id 0x01002927");
}

void RP2040OCDTarget::initNormal() {
    initCommon();

    // Create RP2040 DAP and core 0 target
    // Taken from OpenOCD target/rp2040.cfg
    openocd->sendCommand("dap create rp2040.dap0 -chain-position rp2040.cpu -dp-id 0x01002927 -instance-id 0");
    openocd->sendCommand("target create rp2040.core0 cortex_m -dap rp2040.dap0 -coreid 0");
    openocd->sendCommand("rp2040.core0 cortex_m reset_config sysresetreq");

    openocd->init();

    // Halt the cpu to get it ready to receive commands
    openocd->sendCommand("halt 3000", false, 5000);
    if (openocd->sendCommand("rp2040.core0 curstate") != "halted") {
        throw PicoprobeError("Failed to halt CPU after init");
    }

    // Clear resets on required elements for flash programming
    const uint32_t rst_mask = RESETS_RESET_IO_QSPI_BITS | RESETS_RESET_PADS_QSPI_BITS | RESETS_RESET_TIMER_BITS |
                              RESETS_RESET_IO_BANK0_BITS | RESETS_RESET_PADS_BANK0_BITS;
    writeWord(RESETS_BASE + REG_ALIAS_CLR_BITS, rst_mask);
    if ((readWord(RESETS_BASE) & rst_mask) != 0) {
        throw PicoprobeError("Failed to un-reset required subsystems");
    }

    // Disable pull resistors on UART port for Mk 2 electronics (GPIO16)
    // This prevents garbage from appearing in the serial terminal during upload
    const uint32_t pull_en_mask = PADS_BANK0_GPIO16_PDE_BITS | PADS_BANK0_GPIO16_PUE_BITS;
    writeWord(PADS_BANK0_BASE + REG_ALIAS_CLR_BITS + PADS_BANK0_GPIO16_OFFSET, pull_en_mask);
    if ((readWord(RESETS_BASE + PADS_BANK0_GPIO16_OFFSET) & pull_en_mask) != 0) {
        throw PicoprobeError("Failed to clear pull resistors on GPIO16");
    }
}

void RP2040OCDTarget::doDetach() {
    initCommon();

    // This detaches the DAP so the core doesn't think its still debugged
    // If you don't do this, if a breakpoint is hit, it'll actually stop the CPU instead of firing the watchdog
    openocd->sendCommand("dap create rp2040.dap0 -chain-position rp2040.cpu -dp-id 0x01002927 -instance-id 0");
    openocd->init();
    openocd->sendCommand("rp2040.dap0 dpreg 0x4 0x00000000");
}

void RP2040OCDTarget::initRescue() {
    initCommon();

    // Create and init rescue DAP
    // Taken from OpenOCD target/rp2040.cfg
    openocd->sendCommand("dap create rp2040.rescue_dap -chain-position rp2040.cpu -dp-id 0x01002927 -instance-id 0xf "
                         "-ignore-syspwrupack");
    openocd->init();

    // De-assert rescue signal now that openocd has initialized
    openocd->sendCommand("rp2040.rescue_dap dpreg 0x4 0x00000000");

    // Readback to ensure the de-assert was successful
    auto result = openocd->sendCommand("rp2040.rescue_dap dpreg 0x4");

    const char *parseStart = result.c_str();
    char *parseEnd;
    auto dpreg4 = std::strtoul(parseStart, &parseEnd, 16);

    if (result.size() == 0 || parseStart + result.size() != parseEnd) {
        throw PicoprobeError("Failed to connect to RP2040");
    }

    if (dpreg4 & 0xf0000000) {
        throw PicoprobeError("Failed to de-assert rescue DAP reset");
    }
}

uint32_t RP2040OCDTarget::readWord(uint32_t addr, int width) {
    // Create command to send
    std::stringstream cmdStream;
    cmdStream << "read_memory 0x" << std::hex << addr << std::dec << " " << width << " 1";

    auto result = openocd->sendCommand(cmdStream.str());

    std::stringstream parseStream;
    parseStream << std::hex << result;

    uint32_t value;
    if (!(parseStream >> value)) {
        throw PicoprobeError("Failed to read memory");
    }
    return value;
}

void RP2040OCDTarget::writeWord(uint32_t addr, uint32_t data, int width) {
    // Create command to send
    std::stringstream cmdStream;
    cmdStream << "write_memory 0x" << std::hex << addr << std::dec << " " << width;
    cmdStream << std::hex << " {0x" << data << "}";

    openocd->sendCommand(cmdStream.str());
}

std::vector<uint32_t> RP2040OCDTarget::readMemory(uint32_t addr, uint32_t count, int width) {
    // Create command to send
    std::stringstream cmdStream;
    cmdStream << "read_memory 0x" << std::hex << addr << std::dec << " " << width << " " << count;

    auto result = openocd->sendCommand(cmdStream.str());

    std::vector<uint32_t> data;
    data.reserve(count);
    std::stringstream parseStream;
    parseStream << std::hex << result;

    uint32_t value;
    while (parseStream >> value) {
        data.push_back(value);
    }

    if (data.size() != count) {
        throw PicoprobeError("Failed to read memory");
    }

    return data;
}

void RP2040OCDTarget::writeMemory(uint32_t addr, std::vector<uint32_t> data, int width) {
    std::stringstream cmdStream;
    cmdStream << "write_memory 0x" << std::hex << addr << std::dec << " " << width;
    cmdStream << " {" << std::hex;
    for (uint32_t entry : data) {
        cmdStream << " 0x" << entry;
    }
    cmdStream << "}";

    openocd->sendCommand(cmdStream.str());
}

#define BOOTROM_MAGIC_ADDR 0x00000010
#define BOOTROM_FUNC_TABLE_PTR_ADDR 0x00000014

void RP2040OCDTarget::readoutRom() {
    // Sanity check bootrom
    auto magic = readMemory(BOOTROM_MAGIC_ADDR, 4, 8);
    if (magic.at(0) != 'M' || magic.at(1) != 'u' || magic.at(2) != 1) {
        throw PicoprobeError("Failed to discover RP2040 Bootrom");
    }

    bootromVersion = magic.at(3);
    funcTable.clear();

    uint32_t tablePtr = readWord(BOOTROM_FUNC_TABLE_PTR_ADDR, 16);
    while (true) {
        auto entry = readMemory(tablePtr, 2, 16);
        uint16_t code = entry.at(0);
        uint16_t ptr = entry.at(1);

        if (code == 0)
            break;
        funcTable.emplace(code, ptr);
        tablePtr += 4;
    }
}

uint32_t RP2040OCDTarget::callFunction(uint32_t addr, uint32_t r0, uint32_t r1, uint32_t r2, uint32_t r3,
                                       int timeout_ms) {
    // Construct input register state
    uint32_t bkptAddr = lookupRomFunc('D', 'E');  // _debug_trampoline_end
    addr |= 1;  // Make sure to add the bit to addr to avoid hardfault from non-thumb address

    std::stringstream cmdStream;
    cmdStream << "set_reg {" << std::hex;
    cmdStream << " pc 0x" << addr;
    cmdStream << " r0 0x" << r0;
    cmdStream << " r1 0x" << r1;
    cmdStream << " r2 0x" << r2;
    cmdStream << " r3 0x" << r3;
    cmdStream << " lr 0x" << bkptAddr;
    cmdStream << "}";

    // Run the CPU
    openocd->sendCommand(cmdStream.str());
    openocd->sendCommand("resume");
    openocd->sendCommand("wait_halt " + std::to_string(timeout_ms), false, timeout_ms + 1000);
    std::string curstate = openocd->sendCommand("rp2040.core0 curstate");
    if (curstate != "halted") {
        throw PicoprobeError("CPU failed to halt after function call. Found in state: " + curstate);
    }

    // Parse the current register state
    std::string regstate = openocd->sendCommand("get_reg {pc r0}");
    std::stringstream parseStream;
    parseStream << std::hex << regstate;

    unsigned long newPc, newR0;

    std::string prefix;
    parseStream >> prefix;
    if (prefix != "pc") {
        throw PicoprobeError("Malformed register response: " + regstate);
    }

    parseStream >> newPc;
    // Check equality ignoring LSB
    if ((newPc | 1) != (bkptAddr | 1)) {
        throw PicoprobeError("CPU halted at unexpected addr: " + regstate);
    }

    parseStream >> prefix;
    if (prefix != "r0") {
        throw PicoprobeError("Malformed register response: " + regstate);
    }

    parseStream >> newR0;
    return newR0;
}
