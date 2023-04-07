#include <bit>
#include <iomanip>
#include <iostream>
#include "RP2040FlashInterface.hpp"

namespace UploadTool {

bool showConfirmation(const char *msg) {
    std::cout << msg << " [y/N]: ";
    std::flush(std::cout);

    char input;
    std::cin >> input;

    return input == 'y' || input == 'Y';
}

void drawProgressBar(float percentage) {
    if (percentage > 1.0) percentage = 1.0;
    const int barWidth = 40;

    std::cout << "[";
    int pos = barWidth * percentage;
    for (int i = 0; i < barWidth; ++i) {
        if (i < pos) std::cout << "=";
        else if (i == pos) std::cout << ">";
        else std::cout << " ";
    }
    std::cout << "] " << int(percentage * 100.0) << " %\r";
    std::cout.flush();
}

#define COLOR_TITLE "\033[1;35m"
#define COLOR_ID "\033[0m"
#define COLOR_HEADER "\033[1;90m"
#define COLOR_NAME "\033[0;33m"
#define COLOR_BODY "\033[0;32m"
#define COLOR_RESET "\033[0m"

void printDevices(std::vector<std::shared_ptr<RP2040Device>> &discoveredDevices, DeviceMap &deviceMap) {
    std::cout << COLOR_TITLE "==========Target Selection==========" COLOR_RESET << std::endl;
    for (unsigned int i = 0; i < discoveredDevices.size(); i++) {
        auto dev = discoveredDevices.at(i);
        auto devDescr = deviceMap.lookupSerial(dev->getFlashId());
        // Device ID
        std::cout << COLOR_ID "Dev " << i + 1 << ":\t";

        // Device Name
        std::cout << COLOR_HEADER "Name:\t\t";
        std::cout << COLOR_NAME << devDescr.name << COLOR_RESET << std::endl;

        std::cout << COLOR_HEADER "\tMode:\t\t";
        std::cout << COLOR_BODY << dev->getMode() << COLOR_RESET << std::endl;

        std::cout << COLOR_HEADER "\tInterface:\t";
        std::cout << COLOR_BODY << dev->getInterface() << COLOR_RESET << std::endl;

        std::cout << COLOR_HEADER "\tBoard Type:\t";
        std::cout << COLOR_BODY << devDescr.boardType << COLOR_RESET << std::endl;

        std::cout << COLOR_HEADER "\tUnique ID:\t";
        std::cout << COLOR_BODY << devDescr.hexSerialNum() << COLOR_RESET << std::endl;

        std::cout << std::endl;
    }
}

std::shared_ptr<RP2040FlashInterface> selectInterface(std::vector<std::shared_ptr<RP2040Device>> &discoveredDevices, DeviceMap &deviceMap) {
    if (discoveredDevices.size() == 0) {
        throw std::runtime_error("No devices to select");
    }

    printDevices(discoveredDevices, deviceMap);
    while (1) {
        std::cout << "Please Select a Device: ";

        unsigned int userChoice;
        std::cin >> userChoice;

        if(std::cin.fail()) {
            std::cout << "Invalid input!" << std::endl;
            std::cin.clear();
            std::cin.ignore(256,'\n');
            continue;
        }

        if (userChoice < 1 || userChoice > discoveredDevices.size()) {
            std::cout << "Invalid input!" << std::endl;
            continue;
        }

        return discoveredDevices.at(userChoice - 1)->getFlashInterface();
    }
}

void flashImage(RP2040FlashInterface &interface, RP2040UF2 &uf2) {
    // Check with user to confirm they want to overwrite bootloader
    if (!uf2.isOTA && interface.shouldWarnOnBootloaderOverwrite()) {
        if (!showConfirmation("[WARNING!] Attempting to flash/overwrite the bootloader via the bootloader interface! This may brick the interface!\nAre you CERTAIN you want to continue flashing the full image?")) {
            std::cout << "Flash aborted." << std::endl;
            return;
        }
        interface.enableBootloaderOverwrite();
    }

    // Check if image is a full flash image (overwriting application boot2), and if so perform additional checks
    uint32_t appBoot2Addr = FLASH_BASE + BOOTLOADER_SIZE;
    if (uf2.isOTA && uf2.getBaseAddress() == appBoot2Addr) {
        // Make sure boot2 matches (since pico sdk likes to refer to XIP_BASE rather than the linker boot2)
        // So in the event the sdk calls boot2 (and it typically always does), it won't break things
        auto appBoot2 = uf2.getAddress(appBoot2Addr);

        if (!interface.verifyBytes(FLASH_BASE, appBoot2)) {
            if (!showConfirmation("[WARNING] The boot2 used by this application does not match the bootloader boot2. This may cause performance or other stability issues. Continue with flashing?")) {
                std::cout << "Flash aborted." << std::endl;
                return;
            }
        }

        // TODO: Check that there is a bootloader there if trying to flash ota
    }

    // Write all blocks except for first
    // This allows for the application to be verified before sealing (w/ boot2)
    std::cout << "Writing Image..." << std::endl;
    uint32_t blockCount = uf2.getNumBlocks();
    for (uint32_t i = 1; i < blockCount; i++) {
        // Draw progress
        drawProgressBar(((float)i) / (blockCount - 1));

        // Write data
        auto block = uf2.getBlock(i);
        interface.writeBytes(uf2.getBlockAddress(i), block);
    }
    drawProgressBar(1.0);

    std::cout << std::endl << "Verifying Image..." << std::endl;
    for (uint32_t i = 1; i < blockCount; i++) {
        // Draw progress
        drawProgressBar(((float)i) / (blockCount - 1));

        // Write data
        auto block = uf2.getBlock(i);
        if (!interface.verifyBytes(uf2.getBlockAddress(i), block)) {
            std::cout << std::endl << "Verify failed at address 0x" << std::hex << uf2.getBlockAddress(i) << std::dec << std::endl;
            return;
        }
    }
    drawProgressBar(1.0);
    std::cout << std::endl;

    std::cout << "Sealing Image..." << std::endl;
    {
        auto block = uf2.getBlock(0);
        interface.writeBytes(uf2.getBlockAddress(0), block);
        if (!interface.verifyBytes(uf2.getBlockAddress(0), block)) {
            std::cout << "Verify failed at address 0x" << std::hex << uf2.getBlockAddress(0) << std::dec << std::endl;
            return;
        }
    }

    std::cout << "Image Written." << std::endl;
    interface.reboot();
}

}