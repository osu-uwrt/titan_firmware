#include "CLIBackends.hpp"
#include "CLIInterface.hpp"
#include "DeviceMap.hpp"
#include "FileTransferTask.hpp"
#include "RemoteTTYClientTask.hpp"
#include "canmore_cpp/LinuxClient.hpp"

#include <iostream>
#include <stdlib.h>
#include <thread>

using namespace std::chrono_literals;

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

class LinuxFileUploadCommand : public CLICommandHandler<Canmore::LinuxClient> {
public:
    LinuxFileUploadCommand(): CLICommandHandler("upload") {}

    std::string getArgList() const override { return "[src_file], [optional dst_file]"; }
    std::string getHelp() const override { return "Uploads a file to the remote client."; }

    void callback(CLIInterface<Canmore::LinuxClient> &interface, std::vector<std::string> const &args) override {
        // find file to upload on local system, and destination on remote system
        if (args.size() < 1 || args.size() > 2) {
            std::string err = COLOR_ERROR "Expected 1-2 arguments but got ";
            err += std::to_string(args.size());
            err += COLOR_RESET;
            interface.writeLine(err);
            return;
        }

        std::string src_file = args.at(0), dst_file = FileTransferTask::fileName(src_file);
        if (args.size() == 2) {
            dst_file = args.at(1);
        }

        task.upload(interface, src_file, dst_file);
    }

private:
    FileTransferTask task;
};

class LinuxFileDownloadCommand : public CLICommandHandler<Canmore::LinuxClient> {
public:
    LinuxFileDownloadCommand(): CLICommandHandler("download") {}

    std::string getArgList() const override { return "[src_file], [optional dst_file]"; }
    std::string getHelp() const override { return "Downloads a file from the remote client."; }

    void callback(CLIInterface<Canmore::LinuxClient> &interface, std::vector<std::string> const &args) override {
        // find file to download on remote system and destination on local system
        if (args.size() < 1 || args.size() > 2) {
            std::string err = COLOR_ERROR "Expected 1-2 arguments but got ";
            err += std::to_string(args.size());
            err += COLOR_RESET;
            interface.writeLine(err);
            return;
        }

        std::string src_file = args.at(0), dst_file = FileTransferTask::fileName(src_file);
        if (args.size() == 2) {
            dst_file = args.at(1);
        }

        task.download(interface, src_file, dst_file);
    }

private:
    FileTransferTask task;
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

class LinuxSystemCmdPrefix : public CLICommandPrefixHandler<Canmore::LinuxClient> {
public:
    LinuxSystemCmdPrefix(): CLICommandPrefixHandler('!') {}

    void callback(CLIInterface<Canmore::LinuxClient> &interface, std::vector<std::string> const &args) override {
        if (args.size() == 0) {
            interface.writeLine("Cannot execute empty command");
            return;
        }

        // Create command to execute
        std::stringstream oss;
        bool first = false;
        for (auto &arg : args) {
            if (!first) {
                oss << ' ';
            }
            first = false;
            oss << arg;
        }

        // Execute Command
        std::string cmd = oss.str();
        int returncode = system(cmd.c_str());

        // Report if error occurred
        if (returncode != 0) {
            interface.writeLine(COLOR_NOTICE "Command exited with non-zero status code: " + std::to_string(returncode));
        }
    }

    void showHelp(CLIInterface<Canmore::LinuxClient> &interface, bool showUsage) override {
        if (showUsage) {
            std::stringstream oss;
            oss << "Usage: " << prefixChar << "[Command to Execute]";
            interface.writeLine(oss.str());
            return;
        }

        interface.writeLine("");
        interface.writeLine(COLOR_NAME "! Prefix" COLOR_RESET);
        interface.writeLine("\tExecutes command with the system shell");
        interface.writeLine("\tExample: !ls");
    }

private:
    bool remoteHelpFetched = false;
    std::string remoteHelpMsg;
};

LinuxCLI::LinuxCLI(std::shared_ptr<Canmore::LinuxClient> handle): CLIInterface(handle) {
    registerCommand(std::make_shared<LinuxVersionCommand>());
    registerCommand(std::make_shared<LinuxRebootCommand>());
    registerCommand(std::make_shared<LinuxDaemonRestartCommand>());
    registerCommand(std::make_shared<LinuxFileUploadCommand>());
    registerCommand(std::make_shared<LinuxFileDownloadCommand>());
    registerCommand(std::make_shared<LinuxRemoteTTYCommand>());
    registerCommandPrefix(std::make_shared<LinuxSystemCmdPrefix>());
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
