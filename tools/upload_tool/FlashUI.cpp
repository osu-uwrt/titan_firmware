#include <bit>
#include <cstring>
#include <iomanip>
#include <iostream>
#include "UploadTool.hpp"
#include "canmore_cpp/Discovery.hpp"

namespace UploadTool {

std::string hexWord(uint32_t word) {
    std::stringstream ss;
    ss << "0x" << std::hex << std::setw(8) << std::setfill('0') << word;
    return ss.str();
}

bool showConfirmation(const char *msg) {
    std::cout << msg << " [y/N]: ";
    std::flush(std::cout);

    std::string input;
    std::getline(std::cin, input);

    return input == "y" || input == "Y";
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

void dumpUF2(RP2040UF2 &uf2) {
    for (auto &app : uf2.apps) {
        std::cout << COLOR_TITLE "==========" << (app.isBootloader ? "Bootloader" : "Application") << " Image==========" COLOR_RESET << std::endl;
        dumpAppInfo(app);
        std::cout << std::endl;
    }
}

void printImageName(BinaryInfo::AppInfo &app) {
    if (app.isBootloader) {
        std::cout << COLOR_HEADER "[Bootloader] " COLOR_RESET;
    }

    if (app.programName.size() == 0) {
        std::cout << COLOR_HEADER "(No Program Name)" COLOR_RESET;
    } else {
        std::cout << COLOR_BODY << app.programName << COLOR_RESET;
    }

    if (app.programVersion.size() > 0) {
        std::cout << " " COLOR_BODY << app.programVersion << COLOR_RESET;
    }
    std::cout << std::endl;
}

void printDevice(unsigned int index, std::shared_ptr<RP2040Device> dev, DeviceMap &deviceMap) {
    auto devDescr = deviceMap.lookupSerial(dev->getFlashId());
    // Device ID
    if (index != 0) {
        std::cout << COLOR_ID "Dev " << index << ":\t";
    } else {
        std::cout << "\t";
    }

    // Device Name
    std::cout << COLOR_HEADER "Name:\t\t";
    std::cout << COLOR_NAME << devDescr.name << COLOR_RESET << std::endl;

    std::cout << COLOR_HEADER "\tInterface:\t";
    std::cout << COLOR_BODY << dev->getInterface() << COLOR_RESET << std::endl;

    std::cout << COLOR_HEADER "\tBoard Type:\t";
    std::cout << COLOR_BODY << devDescr.boardType << COLOR_RESET << std::endl;

    std::cout << COLOR_HEADER "\tUnique ID:\t";
    std::cout << COLOR_BODY << devDescr.hexSerialNum() << COLOR_RESET << std::endl;

    std::vector<std::pair<std::string, std::string>> devInfo;
    dev->getAdditionalInfo(devInfo);

    for (std::pair<std::string, std::string> entry : devInfo) {
        std::cout << COLOR_HEADER "\t" << entry.first << ":\t";
        if (entry.first.size() <= 6) std::cout << "\t";
        std::cout << COLOR_BODY << entry.second << COLOR_RESET << std::endl;
    }

    std::cout << std::endl;
}

std::shared_ptr<RP2040Device> selectDevice(std::vector<std::shared_ptr<RP2040Device>> &discoveredDevices, DeviceMap &deviceMap, std::string &boardType, bool autoSelect) {
    if (discoveredDevices.size() == 0) {
        throw std::runtime_error("No devices to select");
    }

    // First try to autoselect if requested
    if (autoSelect) {
        std::shared_ptr<RP2040Device> matchingDevice = nullptr;

        // First try to see if we can find one device which matches the boardType
        for (std::shared_ptr<RP2040Device> dev : discoveredDevices) {
            auto devDescr = deviceMap.lookupSerial(dev->getFlashId());
            if (devDescr.boardType == boardType) {
                if (matchingDevice == nullptr) {
                    matchingDevice = dev;
                } else {
                    // If this is the second device, invalidate and break from the search
                    matchingDevice = nullptr;
                    break;
                }
            }
        }

        // If we found a valid device, return
        // Note that we check for the supporting flash interface here, so that if a device is present but in an invalid
        // mode, it won't hide it from the user
        if (matchingDevice != nullptr && matchingDevice->supportsFlashInterface()) {
            std::cout << COLOR_TITLE "Auto Selecting Device:" COLOR_RESET << std::endl;
            printDevice(0, matchingDevice, deviceMap);
            return matchingDevice;
        }
    }

    // Couldn't auto select... Break down discovered devices into three categories
    //  1. Good Matches (same board_type)
    //  2. Other matches (supports flash interface)
    //  3. Invalid matches (doesn't support flashing)

    std::vector<std::shared_ptr<RP2040Device>> goodMatches;
    std::vector<std::shared_ptr<RP2040Device>> otherMatches;
    std::vector<std::shared_ptr<RP2040Device>> invalidMatches;
    unsigned int index = 1;

    for (auto dev : discoveredDevices) {
        if (!dev->supportsFlashInterface()) {
            invalidMatches.push_back(dev);
        }
        else {
            auto devDescr = deviceMap.lookupSerial(dev->getFlashId());
            if (devDescr.boardType == boardType) {
                goodMatches.push_back(dev);
            }
            else {
                otherMatches.push_back(dev);
            }
        }
    }

    if (goodMatches.size() > 0) {
        std::cout << COLOR_TITLE "===========Compatible Targets===========" COLOR_RESET << std::endl;
        for (auto dev : goodMatches) {
            printDevice(index++, dev, deviceMap);
        }
    }

    if (otherMatches.size() > 0) {
        std::cout << COLOR_TITLE "=============Other Targets=============" COLOR_RESET << std::endl;
        for (auto dev : otherMatches) {
            printDevice(index++, dev, deviceMap);
        }
    }

    if (invalidMatches.size() > 0) {
        std::cout << COLOR_TITLE "=========Non-Flashable Targets=========" COLOR_RESET << std::endl;
        for (auto dev : invalidMatches) {
            printDevice(0, dev, deviceMap);
        }
    }

    while (1) {
        std::cout << "Please Select a Device: ";

        if (std::cin.peek() == '\n') {
            // Clear from buffer
            char c;
            std::cin.read(&c, 1);

            // No default option
            continue;
        }

        unsigned int userChoice;
        std::cin >> userChoice;

        if(std::cin.fail()) {
            std::cout << "Invalid input!" << std::endl;
            std::cin.clear();
            std::cin.ignore(256,'\n');
            continue;
        }
        std::cin.ignore(256,'\n');

        if (userChoice < 1 || userChoice >= index) {
            std::cout << "Invalid input!" << std::endl;
            continue;
        }

        // Switch choice to index
        userChoice--;
        if (userChoice < goodMatches.size()) {
            return goodMatches.at(userChoice);
        } else {
            std::cout << "[WARNING] The selected board does not match the expected type '" << boardType << "'. ";
            if (!showConfirmation("Continue?")) {
                continue;
            }
            return otherMatches.at(userChoice - goodMatches.size());
        }
    }
}

std::shared_ptr<RP2040FlashInterface> catchInBootDelay(std::vector<std::shared_ptr<RP2040Discovery>> discoverySources, DeviceMap &deviceMap, RP2040UF2 &uf2) {
    std::cout << COLOR_TITLE "===========Available Interfaces===========" COLOR_RESET << std::endl;
    unsigned int index = 1;
    std::vector<std::shared_ptr<Canmore::Discovery>> validInterfaces;
    for (auto discovery : discoverySources) {
        std::shared_ptr<Canmore::Discovery> canmoreItf = std::dynamic_pointer_cast<Canmore::Discovery>(discovery);
        if (canmoreItf == nullptr) {
            continue;
        }

        // Device ID
        std::cout << "Itf " << index++ << ":\t";

        // Device Name
        std::cout << "Name:\t";
        std::cout << COLOR_NAME << canmoreItf->getInterfaceName() << COLOR_RESET << std::endl;
        validInterfaces.push_back(canmoreItf);
    }

    std::shared_ptr<Canmore::Discovery> selectedInterface = nullptr;
    uint8_t clientId = 0;

    while (1) {
        std::cout << std::endl << "Please Select a Interface: ";

        if (std::cin.peek() == '\n') {
            // Clear from buffer
            char c;
            std::cin.read(&c, 1);

            // No default option
            continue;
        }

        unsigned int userChoice;
        std::cin >> userChoice;

        if(std::cin.fail()) {
            std::cout << "Invalid input!" << std::endl;
            std::cin.clear();
            std::cin.ignore(256,'\n');
            continue;
        }
        std::cin.ignore(256,'\n');

        if (userChoice < 1 || userChoice >= index) {
            std::cout << "Invalid input!" << std::endl;
            continue;
        }

        // Switch choice to index
        selectedInterface = validInterfaces.at(userChoice - 1);
        break;
    }

    uint8_t defaultClientId = 0;
    for (auto &app : uf2.apps) {
        if (app.clientIds.size() == 1) {
            defaultClientId = app.clientIds.at(0);
            if (defaultClientId >= 255 || defaultClientId == 0) {
                std::cout << "Invalid client ID decoded from image: " << std::to_string(defaultClientId) << std::endl;
                defaultClientId = 0;
            }
            break;
        }
        else if (app.clientIds.size() > 1) {
            std::cout << "Provided Firmware Image has multiple valid CAN IDs:" << std::endl;
            for (auto id : app.clientIds) {
                std::cout << "   -" << id << std::endl;
            }
            break;
        }
    }

    while (1) {
        if (defaultClientId == 0) {
            std::cout << "Enter client id to wait for: ";
        }
        else {
            std::cout << "Enter client id to wait for [Detected Default=" << std::to_string(defaultClientId) << "]: ";
        }

        if (std::cin.peek() == '\n') {
            // Clear from buffer
            char c;
            std::cin.read(&c, 1);
            if (defaultClientId == 0) {
                continue;
            }

            // We have a valid default option, pick that one
            // Perform defult option
            clientId = defaultClientId;
            std::cout << "Selecting default id " << std::to_string(defaultClientId) << std::endl;
            break;
        }

        unsigned int userChoice;
        std::cin >> userChoice;

        if(std::cin.fail()) {
            std::cout << "Invalid input!" << std::endl;
            std::cin.clear();
            std::cin.ignore(256,'\n');
            continue;
        }
        std::cin.ignore(256,'\n');

        if (userChoice < 1 || userChoice >= 255) {
            std::cout << "Invalid input!" << std::endl;
            continue;
        }

        // Switch choice to index
        clientId = userChoice;
        break;
    }

    std::cout << "Waiting for device..." << std::endl;
    auto dev = selectedInterface->catchDeviceInBootDelay(clientId);
    auto devDescr = deviceMap.lookupSerial(dev->getFlashId());

    // Device Name
    std::cout << COLOR_TITLE "Found Device:" COLOR_RESET << std::endl;
    std::cout << COLOR_HEADER "\tName:\t\t";
    std::cout << COLOR_NAME << devDescr.name << COLOR_RESET << std::endl;

    std::cout << COLOR_HEADER "\tInterface:\t";
    std::cout << COLOR_BODY << selectedInterface->getInterfaceName() << COLOR_RESET << std::endl;

    std::cout << COLOR_HEADER "\tBoard Type:\t";
    std::cout << COLOR_BODY << devDescr.boardType << COLOR_RESET << std::endl;

    std::cout << COLOR_HEADER "\tUnique ID:\t";
    std::cout << COLOR_BODY << devDescr.hexSerialNum() << COLOR_RESET << std::endl;

    std::cout << COLOR_HEADER "\tBL Version:\t";
    std::cout << COLOR_BODY << dev->getVersion() << COLOR_RESET << std::endl;

    std::cout << std::endl;

    if (devDescr.boardType != uf2.boardType) {
        std::cout << "[WARNING] The discovered board does not match the expected type '" << uf2.boardType << "'. ";
        if (!showConfirmation("Continue?")) {
            throw std::runtime_error("Flash Aborted");
        }
    }

    return dev;
}

void flashImage(std::shared_ptr<RP2040FlashInterface> interface, RP2040UF2 &uf2, bool isOTA) {
    // Error if the image is large for flash
    if (uf2.getBaseFlashOffset() + uf2.getSize() > interface->getFlashSize()) {
        std::cout << "[ERROR] Requested image @+0x" << std::hex << uf2.getBaseFlashOffset() << " len 0x" << uf2.getSize()
                  << " is too large to fit into device flash size 0x" << interface->getFlashSize()
                  << std::dec << std::endl;
        return;
    }

    // Error if uf2 reports OTA but tries to overwrite bootloader
    if (isOTA && uf2.getBaseFlashOffset() < interface->tryGetBootloaderSize()) {
        std::cout << "[ERROR] Requested OTA image @+0x" << std::hex << uf2.getBaseFlashOffset()
                  << " overlaps with bootloader reserved space @+0x" << interface->tryGetBootloaderSize()
                  << std::dec << std::endl;
        return;
    }

    // Check with user to confirm they want to overwrite bootloader
    if (!isOTA && interface->shouldWarnOnBootloaderOverwrite()) {
        if (!showConfirmation("[WARNING!] Attempting to flash/overwrite the bootloader via the bootloader interface! This may brick the interface!\nAre you CERTAIN you want to continue flashing the full image?")) {
            std::cout << "Flash aborted." << std::endl;
            return;
        }
        interface->enableBootloaderOverwrite();
    }

    // Warn if trying to flash OTA without a bootloader
    if (isOTA && uf2.getBaseFlashOffset() != 0 && interface->tryGetBootloaderSize() == 0) {
        if (!showConfirmation("[WARNING] Attempting to flash OTA image but no bootloader was found. Continue with flashing?")) {
            std::cout << "Flash aborted." << std::endl;
            return;
        }
    }

    // Check if image is a full flash image (overwriting application boot2), and if so perform additional checks
    uint32_t appBoot2Off = interface->tryGetBootloaderSize();
    if (isOTA && uf2.getBaseFlashOffset() != 0 && uf2.getBaseFlashOffset() == appBoot2Off) {
        // Make sure boot2 matches (since pico sdk likes to refer to XIP_BASE rather than the linker boot2)
        // So in the event the sdk calls boot2 (and it typically always does), it won't break things
        auto appBoot2 = uf2.getFlashOffset(appBoot2Off);

        if (!interface->verifyBytes(FLASH_BASE, appBoot2)) {
            if (!showConfirmation("[WARNING] The boot2 used by this application does not match the bootloader boot2. This may cause performance or other stability issues. Continue with flashing?")) {
                std::cout << "Flash aborted." << std::endl;
                return;
            }
        }
    }

    // Report which image we're flashing
    if (isOTA && uf2.getBaseFlashOffset() != 0) {
        std::cout << std::endl << "Flashing OTA Image:";
    } else {
        std::cout << std::endl << "Flashing Full Image:";
    }
    if (uf2.apps.size() == 0) {
        std::cout << COLOR_HEADER " (No Binary Info Found)" COLOR_RESET << std::endl;
    }
    else if (uf2.apps.size() == 1) {
        std::cout << " ";
        printImageName(uf2.apps.at(0));
    }
    else {
        std::cout << std::endl;
        for (auto &app : uf2.apps) {
            std::cout << " - ";
            printImageName(app);
        }
    }
    std::cout << std::endl;

    // Create erase wrapper to handle the erasing of blocks for us
    RP2040EraseWrapper flashWrapper(interface);

    // Write all blocks except for first
    // This allows for the application to be verified before sealing (w/ boot2)
    // By writing this way, if the upload is interrupted, the previous stage (whether it be bootrom or bootloader)
    // will notice that the boot2 has an invalid crc (since we can only erase in 4KB pages, so boot2 will be erased when)
    // the second block (@256 bytes) is written. It will then refuse to run, and fallback to accepting a new image
    std::cout << "Writing Image..." << std::endl;
    uint32_t blockCount = uf2.getNumBlocks();
    for (uint32_t i = 1; i < blockCount; i++) {
        // Draw progress
        drawProgressBar(((float)i) / (blockCount - 1));

        // Write data
        auto block = uf2.getBlock(i);
        flashWrapper.writeBytes(uf2.getBlockAddress(i), block);
    }
    drawProgressBar(1.0);

    std::cout << std::endl << "Verifying Image..." << std::endl;
    for (uint32_t i = 1; i < blockCount; i++) {
        // Draw progress
        drawProgressBar(((float)i) / (blockCount - 1));

        // Write data
        auto block = uf2.getBlock(i);
        if (!interface->verifyBytes(uf2.getBlockAddress(i), block)) {
            std::cout << std::endl << "Verify failed at address 0x" << std::hex << uf2.getBlockAddress(i) << std::dec << std::endl;
            return;
        }
    }
    drawProgressBar(1.0);
    std::cout << std::endl;

    std::cout << "Sealing Image..." << std::endl;
    {
        auto block = uf2.getBlock(0);
        flashWrapper.writeBytes(uf2.getBlockAddress(0), block);
        if (!interface->verifyBytes(uf2.getBlockAddress(0), block)) {
            std::cout << "Verify failed at address 0x" << std::hex << uf2.getBlockAddress(0) << std::dec << std::endl;
            return;
        }
    }

    std::cout << "Image Written." << std::endl;
    interface->reboot();
}

}
