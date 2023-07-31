#include <iostream>
#include <sstream>
#include <thread>
#include "canmore_cpp/DebugClient.hpp"
#include "CLIBackends.hpp"
#include "CLIInterface.hpp"
#include "DeviceMap.hpp"

static void reportNotice(CLIInterface<Canmore::DebugClient> &interface, std::string message) {
    auto uptime = interface.handle->getUptime();
    std::stringstream outStream;
    outStream << COLOR_NOTICE << "[" << uptime.formatSeconds() << "] " << message << COLOR_RESET;
    interface.writeLine(outStream.str());
}

class AppKeepaliveTask: public CLIBackgroundTask<Canmore::DebugClient> {
public:
    AppKeepaliveTask(): hasRun(false) {}

    void callback(CLIInterface<Canmore::DebugClient> &interface) override {
        auto safetyStatus = interface.handle->getSafetyStatus();
        uint32_t faultState = interface.handle->getActiveFaults();
        if (hasRun) {
            if (lastSafetyInitialized != safetyStatus.safetyInitialized) {
                if (safetyStatus.safetyInitialized) {
                    reportNotice(interface, "Safety Initialized (ROS Connected)");
                }
                else {
                    reportNotice(interface, "Safety Deinitialized (ROS Disconnected)");
                }
            }
            if (lastKillIsEnabled != safetyStatus.killIsEnabled) {
                if (safetyStatus.killIsEnabled) {
                    reportNotice(interface, "Kill Switch Inserted");
                }
                else {
                    reportNotice(interface, "Kill Switch Removed");
                }
            }

            uint32_t changedFaults = faultState ^ lastFaultState;
            if (changedFaults != 0) {
                for (uint32_t i = 0; i < 32; i++) {
                    if (changedFaults & (1ul << i)) {
                        if (faultState & (1ul << i)) {
                            reportNotice(interface, "Fault " + interface.handle->lookupFaultName(i) + " (" + std::to_string(i) + ") Raised");
                        }
                        else {
                            reportNotice(interface, "Fault " + interface.handle->lookupFaultName(i) + " (" + std::to_string(i) + ") Lowered");
                        }
                    }
                }
            }
        }
        else {
            hasRun = true;
        }
        lastKillIsEnabled = safetyStatus.killIsEnabled;
        lastSafetyInitialized = safetyStatus.safetyInitialized;
        lastFaultState = faultState;
    }

private:
    bool hasRun;
    bool lastKillIsEnabled;
    bool lastSafetyInitialized;
    uint32_t lastFaultState;
};

class AppEnterBootloaderCommand: public CanmoreCommandHandler<Canmore::DebugClient> {
public:
    AppEnterBootloaderCommand(): CanmoreCommandHandler("enterbl") {}

    std::string getArgList() const override {return "";}
    std::string getHelp() const override {return "Reboots the device into bootloader mode.";}

    void callbackSafe(CLIInterface<Canmore::DebugClient> &interface, std::vector<std::string> const& args) override {
        (void) args;
        interface.writeLine("Requesting reboot into bootloader mode...");
        interface.handle->enterBootloader();
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        interface.close("Closed due to Reboot Request");
    }
};

class AppCrashlogCommand: public CanmoreCommandHandler<Canmore::DebugClient> {
public:
    AppCrashlogCommand(): CanmoreCommandHandler("crashlog") {}

    std::string getArgList() const override {return "";}
    std::string getHelp() const override {return "Returns the full device crash log.";}

    void callbackSafe(CLIInterface<Canmore::DebugClient> &interface, std::vector<std::string> const& args) override {
        (void) args;
        auto crashcounter = interface.handle->getCrashCounter();
        std::vector<Canmore::CrashLogEntry> crashlog;
        interface.handle->getCrashLog(crashlog);

        renderHeader("Crash Log");
        renderName(crashcounter.format());
        int i = 1;
        for (auto entry : crashlog) {
            renderField("Reset -" + std::to_string(i++), entry.format());
        }
    }
};

class AppSafetyStatusCommand: public CanmoreCommandHandler<Canmore::DebugClient> {
public:
    AppSafetyStatusCommand(): CanmoreCommandHandler("safetystatus") {}

    std::string getArgList() const override {return "";}
    std::string getHelp() const override {return "Returns the current safety status.";}

    void callbackSafe(CLIInterface<Canmore::DebugClient> &interface, std::vector<std::string> const& args) override {
        (void) args;
        auto safetyStatus = interface.handle->getSafetyStatus();
        renderField("Safety Setup", (safetyStatus.safetySetup ? "Yes" : "No"));
        renderField("Safety Inited", (safetyStatus.safetyInitialized ? "Yes" : "No"));
        renderField("Kill State", (safetyStatus.killIsEnabled ? "Enabled" : "Killed"));
        renderField("Error State", (safetyStatus.faultPresent ? COLOR_ERROR "Fault Present" COLOR_RESET : "Normal"));
    }
};

class AppFaultsCommand: public CanmoreCommandHandler<Canmore::DebugClient> {
public:
    AppFaultsCommand(): CanmoreCommandHandler("faults") {}

    std::string getArgList() const override {return "";}
    std::string getHelp() const override {return "Returns the list of active faults.";}

    void callbackSafe(CLIInterface<Canmore::DebugClient> &interface, std::vector<std::string> const& args) override {
        (void) args;
        uint32_t activeFaults = interface.handle->getActiveFaults();
        if (activeFaults == 0) {
            renderName("No faults active");
        }
        else {
            renderHeader("Active Faults");
            for (uint32_t i = 0; i < 32; i++) {
                if (activeFaults & (1ul << i)) {
                    renderField("Fault " + std::to_string(i), interface.handle->lookupFaultName(i));
                }
            }
        }
    }
};

class AppRaiseFaultCommand: public CanmoreCommandHandler<Canmore::DebugClient> {
public:
    AppRaiseFaultCommand(): CanmoreCommandHandler("raisefault") {}

    std::string getArgList() const override {return "[fault id]";}
    std::string getHelp() const override {return "Raises the requested fault id.";}

    void callbackSafe(CLIInterface<Canmore::DebugClient> &interface, std::vector<std::string> const& args) override {
        if (args.size() != 1) {
            interface.writeLine("Expected 1 argument");
            interface.showHelp(commandName, true);
            return;
        }

        uint32_t faultId;
        try {
            faultId = std::stoi(args.at(0), nullptr, 10);
        } catch (std::exception &e) {
            interface.writeLine("Invalid fault id specified, expected integer <32.");
            return;
        }

        if (faultId > 31) {
            interface.writeLine("Invalid fault id specified, expected integer <32.");
            return;
        }

        interface.writeLine("Raising fault " + interface.handle->lookupFaultName(faultId));
        interface.handle->raiseFault(faultId);
    }
};

class AppLowerFaultCommand: public CanmoreCommandHandler<Canmore::DebugClient> {
public:
    AppLowerFaultCommand(): CanmoreCommandHandler("lowerfault") {}

    std::string getArgList() const override {return "[fault id]";}
    std::string getHelp() const override {return "Lowers the requested fault id.";}

    void callbackSafe(CLIInterface<Canmore::DebugClient> &interface, std::vector<std::string> const& args) override {
        if (args.size() != 1) {
            interface.writeLine("Expected 1 argument");
            interface.showHelp(commandName, true);
            return;
        }

        uint32_t faultId;
        try {
            faultId = std::stoi(args.at(0), nullptr, 10);
        } catch (std::exception &e) {
            interface.writeLine("Invalid fault id specified, expected integer <32.");
            return;
        }

        if (faultId > 31) {
            interface.writeLine("Invalid fault id specified, expected integer <32.");
            return;
        }

        interface.writeLine("Lowering fault " + interface.handle->lookupFaultName(faultId));
        interface.handle->lowerFault(faultId);
    }
};


class AppFaultNamesCommand: public CanmoreCommandHandler<Canmore::DebugClient> {
public:
    AppFaultNamesCommand(): CanmoreCommandHandler("faultnames") {}

    std::string getArgList() const override {return "";}
    std::string getHelp() const override {return "Returns the list of all fault names.";}

    void callbackSafe(CLIInterface<Canmore::DebugClient> &interface, std::vector<std::string> const& args) override {
        (void) args;
        renderHeader("All Fault Names:");
        for (uint32_t i = 0; i < 32; i++) {
            renderField("Fault " + std::to_string(i), interface.handle->lookupFaultName(i));
        }
    }
};

class AppUptimeCommand: public CanmoreCommandHandler<Canmore::DebugClient> {
public:
    AppUptimeCommand(): CanmoreCommandHandler("uptime") {}

    std::string getArgList() const override {return "";}
    std::string getHelp() const override {return "Returns the current uptime.";}

    void callbackSafe(CLIInterface<Canmore::DebugClient> &interface, std::vector<std::string> const& args) override {
        (void) args;
        renderField("Uptime", interface.handle->getUptime().format());
    }
};

class AppVersionCommand: public CanmoreCommandHandler<Canmore::DebugClient> {
public:
    AppVersionCommand(): CanmoreCommandHandler("version") {}

    std::string getArgList() const override {return "";}
    std::string getHelp() const override {return "Returns the current application version.";}

    void callbackSafe(CLIInterface<Canmore::DebugClient> &interface, std::vector<std::string> const& args) override {
        (void) args;
        renderField("App Version", interface.handle->getVersion());
    }
};

class AppRebootCommand: public CanmoreCommandHandler<Canmore::DebugClient> {
public:
    AppRebootCommand(): CanmoreCommandHandler("reboot") {}

    std::string getArgList() const override {return "";}
    std::string getHelp() const override {return "Reboots the device.";}

    void callbackSafe(CLIInterface<Canmore::DebugClient> &interface, std::vector<std::string> const& args) override {
        (void) args;
        interface.writeLine("Sending reboot command...");
        interface.handle->reboot();

        // Wait for device to reset
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        interface.close("Closed due to Reboot Request");
    }
};

ApplicationCLI::ApplicationCLI(std::shared_ptr<Canmore::DebugClient> handle): CLIInterface(handle) {
    registerCommand(std::make_shared<AppUptimeCommand>());
    registerCommand(std::make_shared<AppVersionCommand>());
    registerCommand(std::make_shared<AppRebootCommand>());
    registerCommand(std::make_shared<AppEnterBootloaderCommand>());
    registerCommand(std::make_shared<AppCrashlogCommand>());
    registerCommand(std::make_shared<AppFaultsCommand>());
    registerCommand(std::make_shared<AppRaiseFaultCommand>());
    registerCommand(std::make_shared<AppLowerFaultCommand>());
    registerCommand(std::make_shared<AppFaultNamesCommand>());
    registerCommand(std::make_shared<AppSafetyStatusCommand>());
    setBackgroundTask(std::make_shared<AppKeepaliveTask>());

    auto devMap = DeviceMap::create();
    uint64_t flashId = handle->getFlashId();
    auto devDescr = devMap.lookupSerial(flashId);

    std::cout << std::endl;
    renderHeader("Connecting to Application");
    renderName(devDescr.name);
    if (devDescr.boardType != "unknown") renderField("Board Type", devDescr.boardType);
    else if (flashId != 0) renderField("Unique ID", devDescr.hexSerialNum());
    renderField("Version", handle->getVersion());

    auto lastCrash = handle->getLastResetEntry();
    renderField("Reset Cause", lastCrash.format());

    auto uptime = handle->getUptime();
    renderField("Uptime", uptime.format());

    auto crashCounter = handle->getCrashCounter();
    if (crashCounter.totalCount > 0) {
        renderField("Crash Count", crashCounter.format());
    }

    auto safetyStatus = handle->getSafetyStatus();
    renderField("System State", safetyStatus.getGlobalState());
    renderField("Error State", safetyStatus.faultPresent ? COLOR_ERROR "Fault Present" : "Normal");
    std::cout << std::endl;
}
