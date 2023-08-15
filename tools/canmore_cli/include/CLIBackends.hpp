#pragma once

#include "CLIInterface.hpp"
#include "canmore_cpp/BootloaderClient.hpp"
#include "canmore_cpp/DebugClient.hpp"

template <class T> class CanmoreCommandHandler : public CLICommandHandler<T> {
public:
    CanmoreCommandHandler(std::string const &commandName): CLICommandHandler<T>(commandName) {}

    virtual void callbackSafe(CLIInterface<T> &interface, std::vector<std::string> const &args) = 0;

    void callback(CLIInterface<T> &interface, std::vector<std::string> const &args) override final {
        try {
            callbackSafe(interface, args);
        } catch (Canmore::CanmoreError &e) {
            interface.writeLine(COLOR_ERROR "Exception caught while running command:" COLOR_RESET);
            interface.writeLine(COLOR_ERROR "  what(): " + std::string(e.what()) + COLOR_RESET);
        }
    }
};

class BootloaderCLI : public CLIInterface<Canmore::BootloaderClient> {
public:
    BootloaderCLI(std::shared_ptr<Canmore::BootloaderClient> handle);
    std::string getCliName() const override { return "Bootloader"; }
};

class ApplicationCLI : public CLIInterface<Canmore::DebugClient> {
public:
    ApplicationCLI(std::shared_ptr<Canmore::DebugClient> handle);
    std::string getCliName() const override { return "Application"; }
};
