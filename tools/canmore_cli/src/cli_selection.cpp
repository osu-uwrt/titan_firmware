#include "CLIBackends.hpp"
#include "CLIInterface.hpp"
#include "CanmoreCLI.hpp"

#include <iostream>
#include <sstream>
#include <termios.h>
#include <unistd.h>

void reportError(std::string msg) {
    // Backup old flags
    struct termios oldt;
    if (tcgetattr(STDIN_FILENO, &oldt) < 0) {
        throw std::system_error(errno, std::generic_category(), "tcgetattr");
    }

    // Set new flags disabling canonical mode, echo, interrupt signals, and flow control
    struct termios newt = oldt;
    newt.c_lflag &= ~(ICANON | ECHO | ISIG);
    newt.c_iflag &= ~(IXON | IXOFF | IXANY);

    if (tcsetattr(STDIN_FILENO, TCSANOW, &newt) < 0) {
        throw std::system_error(errno, std::generic_category(), "tcsetattr");
    }

    // Report error
    std::cout << std::endl << CURSOR_DISABLE COLOR_ERROR << msg << COLOR_RESET << std::endl;
    std::cout << COLOR_HEADER "Press any enter to continue..." COLOR_RESET << std::flush;
    while (getchar() != '\n')
        ;
    std::cout << CURSOR_ENABLE << std::endl << std::endl;

    // Reset terminal
    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
}

static std::shared_ptr<CLIRunnable> createCliInstance(std::shared_ptr<Canmore::Device> dev) {
    if (auto bootloaderDevice = std::dynamic_pointer_cast<Canmore::BootloaderDevice>(dev)) {
        return std::dynamic_pointer_cast<CLIRunnable>(std::make_shared<BootloaderCLI>(bootloaderDevice->getClient()));
    }
    else if (auto normalDevice = std::dynamic_pointer_cast<Canmore::NormalDevice>(dev)) {
        return std::dynamic_pointer_cast<CLIRunnable>(std::make_shared<ApplicationCLI>(normalDevice->getClient()));
    }
    else if (auto bootDelayDevice = std::dynamic_pointer_cast<Canmore::BootDelayDevice>(dev)) {
        std::cout << "Waiting for 15 seconds for boot delay device to reappear..." << std::endl;
        return std::dynamic_pointer_cast<CLIRunnable>(
            std::make_shared<BootloaderCLI>(bootDelayDevice->waitForBootloader(15000)));
    }
    else if (auto linuxDevice = std::dynamic_pointer_cast<Canmore::LinuxDevice>(dev)) {
        return std::dynamic_pointer_cast<CLIRunnable>(std::make_shared<LinuxCLI>(linuxDevice->getClient()));
    }
    else {
        return nullptr;
    }
}

void runCli(std::shared_ptr<Canmore::Device> dev) {
    try {
        auto cli = createCliInstance(dev);
        if (cli) {
            cli->run();
        }
        else {
            std::stringstream ss;
            ss << "Failed to start CLI!" << std::endl;
            ss << "No compatible CLI backends for device mode '" << dev->getMode() << "'";
            reportError(ss.str());
        }
    } catch (Canmore::CanmoreError &e) {
        std::stringstream ss;
        ss << "Exception while executing CLI!" << std::endl;
        ss << "  what(): " << e.what();
        reportError(ss.str());
    }
    fseek(stdin, 0, SEEK_END);  // Clear stdin buffer before returning to GUI
}

void runCliCommand(std::shared_ptr<Canmore::Device> dev, const std::string &cmd, const std::vector<std::string> &args) {
    try {
        auto cli = createCliInstance(dev);
        if (cli) {
            cli->runCommand(cmd, args);
        }
        else {
            std::stringstream ss;
            ss << "Failed to start CLI!" << std::endl;
            ss << "No compatible CLI backends for device mode '" << dev->getMode() << "'";
            reportError(ss.str());
        }
    } catch (Canmore::CanmoreError &e) {
        std::stringstream ss;
        ss << "Exception while executing CLI!" << std::endl;
        ss << "  what(): " << e.what();
        reportError(ss.str());
    }
    fseek(stdin, 0, SEEK_END);  // Clear stdin buffer before returning to GUI
}
