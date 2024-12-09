#pragma once

#include "CLIInterface.hpp"
#include "canmore_cpp/BootloaderClient.hpp"
#include "canmore_cpp/DebugClient.hpp"
#include "canmore_cpp/LinuxClient.hpp"

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

class LinuxCLI : public CLIInterface<Canmore::LinuxClient> {
public:
    LinuxCLI(std::shared_ptr<Canmore::LinuxClient> handle);
    std::string getCliName() const override { return "Linux"; }
};
