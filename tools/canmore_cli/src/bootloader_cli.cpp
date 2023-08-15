#include "BinaryInfo.hpp"
#include "CLIBackends.hpp"
#include "CLIInterface.hpp"
#include "DeviceMap.hpp"
#include "canmore_cpp/BootloaderClient.hpp"

#include <iostream>
#include <sstream>
#include <thread>

static void DumpHex(const void *data, size_t size) {
    char ascii[17];
    size_t i, j;
    ascii[16] = '\0';
    for (i = 0; i < size; ++i) {
        printf("%02X ", ((unsigned char *) data)[i]);
        if (((unsigned char *) data)[i] >= ' ' && ((unsigned char *) data)[i] <= '~') {
            ascii[i % 16] = ((unsigned char *) data)[i];
        }
        else {
            ascii[i % 16] = '.';
        }
        if ((i + 1) % 8 == 0 || i + 1 == size) {
            printf(" ");
            if ((i + 1) % 16 == 0) {
                printf("|  %s \n", ascii);
            }
            else if (i + 1 == size) {
                ascii[(i + 1) % 16] = '\0';
                if ((i + 1) % 16 <= 8) {
                    printf(" ");
                }
                for (j = (i + 1) % 16; j < 16; ++j) {
                    printf("   ");
                }
                printf("|  %s \n", ascii);
            }
        }
    }
}

class BLKeepaliveTask : public CLIBackgroundTask<Canmore::BootloaderClient> {
public:
    void callback(CLIInterface<Canmore::BootloaderClient> &interface) override { interface.handle->ping(); }
};

class BLBinaryInfoCommand : public CanmoreCommandHandler<Canmore::BootloaderClient> {
public:
    BLBinaryInfoCommand(): CanmoreCommandHandler("binary_info") {}

    std::string getArgList() const override { return ""; }
    std::string getHelp() const override { return "Returns the current binary info for the application."; }

    void callbackSafe(CLIInterface<Canmore::BootloaderClient> &interface,
                      std::vector<std::string> const &args) override {
        (void) args;
        BinaryInfo::AppInfo info;
        BinaryInfo::extractAppInfo(*interface.handle, info, FLASH_BASE + interface.handle->tryGetBootloaderSize());
        renderHeader("Application Info");
        BinaryInfo::dumpAppInfo(info);
    }
};

class BLReadCommand : public CanmoreCommandHandler<Canmore::BootloaderClient> {
public:
    BLReadCommand(): CanmoreCommandHandler("read") {}

    std::string getArgList() const override { return "[address]"; }
    std::string getHelp() const override {
        return "Reads a page at the specified address\nAddress must be a flash address in hex";
    }

    void callbackSafe(CLIInterface<Canmore::BootloaderClient> &interface,
                      std::vector<std::string> const &args) override {
        if (args.size() != 1) {
            interface.writeLine("Expected 1 argument");
            interface.showHelp(commandName, true);
            return;
        }
        auto &addressStr = args.at(0);
        if (addressStr.size() <= 2 || addressStr.at(0) != '0' || addressStr.at(1) != 'x') {
            interface.writeLine("Address must be in hex.");
            return;
        }
        if (addressStr.size() > 10) {
            interface.writeLine("Address must fit within 32-bits");
            return;
        }

        uint32_t address = 0;
        for (size_t i = 2; i != addressStr.length(); i++) {
            uint8_t nibble = 0;
            char hexChar = addressStr.at(i);
            if ('0' <= hexChar && hexChar <= '9') {
                nibble = hexChar - '0';
            }
            else if ('A' <= hexChar && hexChar <= 'F') {
                nibble = hexChar - 'A' + 10;
            }
            else {
                interface.writeLine("Invalid hex character: " + hexChar);
                return;
            }
            address |= ((uint32_t) nibble) << (4 * (addressStr.length() - i - 1));
        }

        if (address < FLASH_BASE || address > (FLASH_BASE + MAX_FLASH_SIZE) || (address % UF2_PAGE_SIZE) != 0) {
            interface.writeLine("Invalid address outside of flash memory");
            return;
        }

        renderHeader("Reading Page @" + addressStr);
        std::array<uint8_t, UF2_PAGE_SIZE> readData;
        interface.handle->readBytes(address, readData);
        DumpHex(readData.data(), readData.size());
    }
};

class BLVersionCommand : public CanmoreCommandHandler<Canmore::BootloaderClient> {
public:
    BLVersionCommand(): CanmoreCommandHandler("version") {}

    std::string getArgList() const override { return ""; }
    std::string getHelp() const override { return "Returns the current application version."; }

    void callbackSafe(CLIInterface<Canmore::BootloaderClient> &interface,
                      std::vector<std::string> const &args) override {
        (void) args;
        renderField("BL Version", interface.handle->getVersion() + COLOR_RESET);
    }
};

class BLRebootCommand : public CanmoreCommandHandler<Canmore::BootloaderClient> {
public:
    BLRebootCommand(): CanmoreCommandHandler("reboot") {}

    std::string getArgList() const override { return ""; }
    std::string getHelp() const override { return "Reboots the device into application"; }

    void callbackSafe(CLIInterface<Canmore::BootloaderClient> &interface,
                      std::vector<std::string> const &args) override {
        (void) args;
        interface.writeLine("Sending reboot command...");
        interface.handle->reboot();
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        interface.close("Closed due to Reboot Request");
    }
};

BootloaderCLI::BootloaderCLI(std::shared_ptr<Canmore::BootloaderClient> handle): CLIInterface(handle) {
    registerCommand(std::make_shared<BLVersionCommand>());
    registerCommand(std::make_shared<BLRebootCommand>());
    registerCommand(std::make_shared<BLBinaryInfoCommand>());
    registerCommand(std::make_shared<BLReadCommand>());
    setBackgroundTask(std::make_shared<BLKeepaliveTask>());

    auto devMap = DeviceMap::create();
    uint64_t flashId = handle->getFlashId();
    auto devDescr = devMap.lookupSerial(flashId);

    std::cout << std::endl;
    renderHeader("Connecting to Bootloader");
    renderName(devDescr.name);
    if (devDescr.boardType != "unknown")
        renderField("Board Type", devDescr.boardType);
    else if (flashId != 0)
        renderField("Unique ID", devDescr.hexSerialNum());
    auto lastPrecision = std::cout.precision(1);
    renderField("Flash Size", std::to_string(handle->getFlashSize() / 1024.0 / 1024.0) + " MB");
    std::cout.precision(lastPrecision);
    renderField("Version", handle->getVersion());
    std::cout << std::endl;
}
