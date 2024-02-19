#include "CLIBackends.hpp"
#include "CLIInterface.hpp"
#include "DeviceMap.hpp"
#include "RemoteTTYClientTask.hpp"
#include "canmore_cpp/LinuxClient.hpp"

#include <iostream>
#include <sstream>
#include <thread>

class LinuxKeepaliveTask : public CLIBackgroundTask<Canmore::LinuxClient> {
public:
    void callback(CLIInterface<Canmore::LinuxClient> &interface) override { interface.handle->ping(); }
};

class LinuxVersionCommand : public CLICommandHandler<Canmore::LinuxClient> {
public:
    LinuxVersionCommand(): CLICommandHandler("version") {}

    std::string getArgList() const override { return ""; }
    std::string getHelp() const override { return "Returns the current version."; }

    void callback(CLIInterface<Canmore::LinuxClient> &interface, std::vector<std::string> const &args) override {
        (void) args;
        renderField("Version", interface.handle->getVersion() + COLOR_RESET);
    }
};

class LinuxRebootCommand : public CLICommandHandler<Canmore::LinuxClient> {
public:
    LinuxRebootCommand(): CLICommandHandler("reboot") {}

    std::string getArgList() const override { return ""; }
    std::string getHelp() const override { return "Performs a full system reboot"; }

    void callback(CLIInterface<Canmore::LinuxClient> &interface, std::vector<std::string> const &args) override {
        (void) args;
        interface.writeLine("Sending reboot command...");
        interface.handle->reboot();
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        interface.close("Closed due to Reboot Request");
    }
};

class LinuxDaemonRestartCommand : public CLICommandHandler<Canmore::LinuxClient> {
public:
    LinuxDaemonRestartCommand(): CLICommandHandler("daemon_restart") {}

    std::string getArgList() const override { return ""; }
    std::string getHelp() const override { return "Restarts the daemon service (if bash manages to break)"; }

    void callback(CLIInterface<Canmore::LinuxClient> &interface, std::vector<std::string> const &args) override {
        (void) args;
        interface.writeLine("Resetting CanmoreCLI Daemon...");
        interface.handle->restartDaemon();
        std::this_thread::sleep_for(std::chrono::milliseconds(2000));
        interface.close("Closed due to daemon stopping");
    }
};

class LinuxRemoteTTYCommand : public CLICommandHandler<Canmore::LinuxClient> {
public:
    LinuxRemoteTTYCommand(): CLICommandHandler("remotesh") {}

    std::string getArgList() const override { return "[optional command]"; }
    std::string getHelp() const override {
        return "Launches a remote shell on the host. If the optional command is provided, that is executed instead of "
               "the login shell.";
    }

    void callback(CLIInterface<Canmore::LinuxClient> &interface, std::vector<std::string> const &args) override {
        (void) args;
        auto canClient = std::dynamic_pointer_cast<Canmore::RegMappedCANClient>(interface.handle->client);
        if (canClient) {
            std::stringstream iss;
            auto itr = args.begin();
            while (itr != args.end()) {
                iss << *itr;
                itr++;
                if (itr != args.end()) {
                    iss << ' ';
                }
            }

            RemoteTTYClientTask ttyClient(interface.handle, canClient, iss.str());
            ttyClient.run();
        }
        else {
            interface.writeLine(COLOR_ERROR "Current interface is not CAN bus, cannot create TTY client" COLOR_RESET);
        }
    }
};

LinuxCLI::LinuxCLI(std::shared_ptr<Canmore::LinuxClient> handle): CLIInterface(handle) {
    registerCommand(std::make_shared<LinuxVersionCommand>());
    registerCommand(std::make_shared<LinuxRebootCommand>());
    registerCommand(std::make_shared<LinuxDaemonRestartCommand>());
    registerCommand(std::make_shared<LinuxRemoteTTYCommand>());
    setBackgroundTask(std::make_shared<LinuxKeepaliveTask>());

    auto devMap = DeviceMap::create();
    uint64_t flashId = handle->getFlashId();
    auto devDescr = devMap.lookupSerial(flashId);

    std::cout << std::endl;
    renderHeader("Connecting to Linux Device");
    renderName(devDescr.name);
    if (devDescr.boardType != "unknown")
        renderField("Board Type", devDescr.boardType);
    else if (flashId != 0)
        renderField("Unique ID", devDescr.hexSerialNum());
    renderField("Version", handle->getVersion());
    std::cout << std::endl;
}
