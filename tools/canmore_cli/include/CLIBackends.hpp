#pragma once

#include "CLIInterface.hpp"
#include "canmore_cpp/BootloaderClient.hpp"
#include "canmore_cpp/DebugClient.hpp"

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
