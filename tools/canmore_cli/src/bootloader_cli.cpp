#include "BinaryInfo.hpp"
#include "CLIBackends.hpp"
#include "CLIInterface.hpp"
#include "DeviceMap.hpp"
#include "GDBServer.hpp"
#include "canmore_cpp/BootloaderClient.hpp"

#include <iostream>
#include <sstream>
#include <thread>

class BLKeepaliveTask : public CLIBackgroundTask<Canmore::BootloaderClient> {
public:
    void callback(CLIInterface<Canmore::BootloaderClient> &interface) override { interface.handle->ping(); }
};

class BLBinaryInfoCommand : public CLICommandHandler<Canmore::BootloaderClient> {
public:
    BLBinaryInfoCommand(): CLICommandHandler("binary_info") {}

    std::string getArgList() const override { return ""; }
    std::string getHelp() const override { return "Returns the current binary info for the application."; }

    void callback(CLIInterface<Canmore::BootloaderClient> &interface, std::vector<std::string> const &args) override {
        (void) args;
        BinaryInfo::AppInfo info;
        BinaryInfo::extractAppInfo(*interface.handle, info, FLASH_BASE + interface.handle->tryGetBootloaderSize());
        renderHeader("Application Info");
        BinaryInfo::dumpAppInfo(info);
    }
};

class BLReadCommand : public CLICommandHandler<Canmore::BootloaderClient> {
public:
    BLReadCommand(): CLICommandHandler("read") {}

    std::string getArgList() const override { return "[address]"; }
    std::string getHelp() const override {
        return "Reads a page at the specified address\nAddress must be a flash address in hex";
    }

    void callback(CLIInterface<Canmore::BootloaderClient> &interface, std::vector<std::string> const &args) override {
        if (args.size() != 1) {
            interface.writeLine("Expected 1 argument");
            interface.showHelp(commandName, true);
            return;
        }

        uint32_t address;
        if (!decodeU32(args.at(0), address)) {
            interface.writeLine("Invalid 32-bit integer provided for address");
            return;
        }

        if (address < FLASH_BASE || address > (FLASH_BASE + MAX_FLASH_SIZE) || (address % UF2_PAGE_SIZE) != 0) {
            interface.writeLine("Invalid address outside of flash memory");
            return;
        }

        renderHeader("Reading Flash Page");
        std::array<uint8_t, UF2_PAGE_SIZE> readData;
        interface.handle->readBytes(address, readData);
        DumpHex(address, readData.data(), readData.size());
    }
};

class BLVersionCommand : public CLICommandHandler<Canmore::BootloaderClient> {
public:
    BLVersionCommand(): CLICommandHandler("version") {}

    std::string getArgList() const override { return ""; }
    std::string getHelp() const override { return "Returns the current application version."; }

    void callback(CLIInterface<Canmore::BootloaderClient> &interface, std::vector<std::string> const &args) override {
        (void) args;
        renderField("BL Version", interface.handle->getVersion() + COLOR_RESET);
    }
};

class BLRebootCommand : public CLICommandHandler<Canmore::BootloaderClient> {
public:
    BLRebootCommand(): CLICommandHandler("reboot") {}

    std::string getArgList() const override { return ""; }
    std::string getHelp() const override { return "Reboots the device into application"; }

    void callback(CLIInterface<Canmore::BootloaderClient> &interface, std::vector<std::string> const &args) override {
        (void) args;
        interface.writeLine("Sending reboot command...");
        interface.handle->reboot();
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        interface.close("Closed due to Reboot Request");
    }
};

class BLGdbServer : public CLICommandHandler<Canmore::BootloaderClient> {
public:
    BLGdbServer(): CLICommandHandler("gdbserver") {}

    const uint16_t default_port = 3333;

    std::string getArgList() const override { return "[port: Default " + std::to_string(default_port) + "]"; }
    std::string getHelp() const override { return "Starts gdb server over Canmore"; }

    void callback(CLIInterface<Canmore::BootloaderClient> &interface, std::vector<std::string> const &args) override {
        uint32_t port = default_port;
        if (args.size() > 0) {
            if (!decodeU32(args.at(0), port, UINT16_MAX)) {
                interface.writeLine("Invalid port number");
                return;
            }
        }

        runGdbServer(port, interface.handle);
    }
};

BootloaderCLI::BootloaderCLI(std::shared_ptr<Canmore::BootloaderClient> handle): CLIInterface(handle) {
    registerCommand(std::make_shared<BLVersionCommand>());
    registerCommand(std::make_shared<BLRebootCommand>());
    registerCommand(std::make_shared<BLBinaryInfoCommand>());
    registerCommand(std::make_shared<BLReadCommand>());
    registerCommand(std::make_shared<BLGdbServer>());
    setBackgroundTask(std::make_shared<BLKeepaliveTask>());

    if (!quietConnect) {
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
}
