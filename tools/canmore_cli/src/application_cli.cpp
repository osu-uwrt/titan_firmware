#include "CLIBackends.hpp"
#include "CLIInterface.hpp"
#include "DeviceMap.hpp"
#include "canmore_cpp/DebugClient.hpp"

#include <iomanip>
#include <iostream>
#include <sstream>
#include <thread>

static void reportNotice(CLIInterface<Canmore::DebugClient> &interface, std::string message) {
    auto uptime = interface.handle->getUptime();
    std::stringstream outStream;
    outStream << COLOR_NOTICE << "[" << uptime.formatSeconds() << "] " << message << COLOR_RESET;
    interface.writeLine(outStream.str());
}

class AppKeepaliveTask : public CLIBackgroundTask<Canmore::DebugClient> {
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
                            reportNotice(interface, "Fault " + interface.handle->lookupFaultName(i) + " (" +
                                                        std::to_string(i) + ") Raised");
                        }
                        else {
                            reportNotice(interface, "Fault " + interface.handle->lookupFaultName(i) + " (" +
                                                        std::to_string(i) + ") Lowered");
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

class AppEnterBootloaderCommand : public CanmoreCommandHandler<Canmore::DebugClient> {
public:
    AppEnterBootloaderCommand(): CanmoreCommandHandler("enterbl") {}

    std::string getArgList() const override { return ""; }
    std::string getHelp() const override { return "Reboots the device into bootloader mode."; }

    void callbackSafe(CLIInterface<Canmore::DebugClient> &interface, std::vector<std::string> const &args) override {
        (void) args;
        interface.writeLine("Requesting reboot into bootloader mode...");
        interface.handle->enterBootloader();
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        interface.close("Closed due to Reboot Request");
    }
};

class AppCrashlogCommand : public CanmoreCommandHandler<Canmore::DebugClient> {
public:
    AppCrashlogCommand(): CanmoreCommandHandler("crashlog") {}

    std::string getArgList() const override { return ""; }
    std::string getHelp() const override { return "Returns the full device crash log."; }

    void callbackSafe(CLIInterface<Canmore::DebugClient> &interface, std::vector<std::string> const &args) override {
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

class AppSafetyStatusCommand : public CanmoreCommandHandler<Canmore::DebugClient> {
public:
    AppSafetyStatusCommand(): CanmoreCommandHandler("safetystatus") {}

    std::string getArgList() const override { return ""; }
    std::string getHelp() const override { return "Returns the current safety status."; }

    void callbackSafe(CLIInterface<Canmore::DebugClient> &interface, std::vector<std::string> const &args) override {
        (void) args;
        auto safetyStatus = interface.handle->getSafetyStatus();
        renderField("Safety Setup", (safetyStatus.safetySetup ? "Yes" : "No"));
        renderField("Safety Inited", (safetyStatus.safetyInitialized ? "Yes" : "No"));
        renderField("Kill State", (safetyStatus.killIsEnabled ? "Enabled" : "Killed"));
        renderField("Error State", (safetyStatus.faultPresent ? COLOR_ERROR "Fault Present" COLOR_RESET : "Normal"));
    }
};

class AppFaultsCommand : public CanmoreCommandHandler<Canmore::DebugClient> {
public:
    AppFaultsCommand(): CanmoreCommandHandler("faults") {}

    std::string getArgList() const override { return "[-a]"; }
    std::string getHelp() const override {
        return "Returns the list of active faults.\nPassing -a requests all faults";
    }

    void callbackSafe(CLIInterface<Canmore::DebugClient> &interface, std::vector<std::string> const &args) override {
        // Command Parsing
        bool show_all = false;
        for (auto &arg : args) {
            if (arg == "-a") {
                show_all = true;
            }
            else {
                interface.writeLine("Unexpected arg: " + arg);
                interface.showHelp(commandName, true);
            }
        }

        // Extract Fault Data
        uint32_t activeFaults = interface.handle->getActiveFaults();
        std::vector<Canmore::FaultData> faultList;
        if (show_all) {
            for (uint32_t i = 0; i < 32; i++) {
                faultList.push_back(interface.handle->lookupFaultData(i));
            }
        }
        else {
            for (uint32_t i = 0; i < 32; i++) {
                if (activeFaults & (1ul << i)) {
                    faultList.push_back(interface.handle->lookupFaultData(i));
                }
            }
        }

        // Render fault data
        if (faultList.size() > 0) {
            FaultRenderer renderer(faultList, show_all);
            std::string headerRow = renderer.renderHeader();

            // Render header
            renderHeader((show_all ? "All Faults" : "Active Faults"), headerRow.length() - 13);  // Subtract ansi codes
            interface.writeLine(headerRow);

            // Render faults
            for (auto row : renderer) {
                interface.writeLine(row);
            }
        }
        else {
            interface.writeLine(COLOR_HEADER "    No faults" COLOR_RESET CLEAR_LINE_AFTER);
        }
    }

    class FaultRenderer {
    public:
        FaultRenderer(const std::vector<Canmore::FaultData> &faultArray, bool showSticky):
            faultArray(faultArray), showSticky(showSticky) {
            // Compute the widths for dynamic fields
            for (auto &entry : faultArray) {
                if (entry.name.size() > nameWidth) {
                    nameWidth = entry.name.size();
                }
                if (entry.filename.size() > filenameWidth) {
                    filenameWidth = entry.filename.size();
                }
                size_t entryTsSize = entry.formatTimestamp().size();
                if (entryTsSize > timestampWidth) {
                    timestampWidth = entryTsSize;
                }
            }
        }

        std::string renderHeader() const {
            std::stringstream out;
            out << COLOR_HEADER << std::left << std::setfill(' ');
            out << " | " << std::setw(faultIdWidth) << "Id";
            out << " | " << std::setw(nameWidth) << "Name";
            if (showSticky) {
                out << " | " << std::setw(stickyWidth) << "Sticky";
            }
            out << " | " << std::setw(timestampWidth) << "Timestamp";
            out << " | " << std::setw(filenameWidth) << "File";
            out << " | " << std::setw(lineWidth) << "Line";
            out << " | " << std::setw(extraDataWidth) << "Extra Data";
            out << " |" COLOR_RESET CLEAR_LINE_AFTER;
            return out.str();
        }

        std::string renderField(const Canmore::FaultData &data) const {
            std::stringstream out;
            out << COLOR_BODY << std::right << std::setfill(' ');
            out << " | " << std::setw(faultIdWidth) << data.faultId << std::setw(0);
            out << " | " << std::setw(nameWidth) << data.name;
            if (showSticky) {
                out << " | " << std::setw(stickyWidth) << (data.sticky ? "Yes" : "No");
            }
            out << " | " << std::setw(timestampWidth) << (data.sticky ? data.formatTimestamp() : "-");
            out << " | " << std::setw(filenameWidth) << (data.sticky ? data.filename : "-");
            out << " | " << std::setw(lineWidth);
            if (data.sticky) {
                out << data.line << std::setw(0);
            }
            else {
                out << "-";
            }
            out << " | ";
            if (data.sticky) {
                if (data.extraData != 0) {
                    out << "0x" << std::setw(extraDataWidth - 2) << std::hex << std::uppercase << std::setfill('0')
                        << data.extraData << std::setw(0) << std::dec << std::setfill(' ');
                }
                else {
                    out << std::setw(extraDataWidth) << "0";
                }
            }
            else {
                out << std::setw(extraDataWidth) << "-";
            }
            out << " |" COLOR_RESET CLEAR_LINE_AFTER;
            return out.str();
        }

        class RendererIterator {
            friend class FaultRenderer;

        public:
            std::string operator*() { return renderer.renderField(*itr); }
            void operator++() { itr++; }
            bool operator!=(const RendererIterator &otherItr) { return otherItr.itr != itr; }

        protected:
            RendererIterator(const FaultRenderer &renderer, const std::vector<Canmore::FaultData>::const_iterator &itr):
                renderer(renderer), itr(itr) {}

        private:
            const FaultRenderer &renderer;
            std::vector<Canmore::FaultData>::const_iterator itr;
        };

        RendererIterator begin() { return RendererIterator(*this, faultArray.begin()); }
        RendererIterator end() { return RendererIterator(*this, faultArray.end()); }

    private:
        std::vector<Canmore::FaultData> faultArray;
        bool showSticky;
        size_t nameWidth = 4;  // Initial values for variable fields taken from header width
        size_t filenameWidth = 4;
        size_t timestampWidth = 9;
        // Constant width fields (either header max or the fixed value length)
        const size_t faultIdWidth = 2;
        const size_t stickyWidth = 6;
        const size_t lineWidth = 6;
        const size_t extraDataWidth = 10;
    };
};

class AppRaiseFaultCommand : public CanmoreCommandHandler<Canmore::DebugClient> {
public:
    AppRaiseFaultCommand(): CanmoreCommandHandler("raisefault") {}

    std::string getArgList() const override { return "[fault id]"; }
    std::string getHelp() const override { return "Raises the requested fault id."; }

    void callbackSafe(CLIInterface<Canmore::DebugClient> &interface, std::vector<std::string> const &args) override {
        if (args.size() != 1) {
            interface.writeLine("Expected 1 argument");
            interface.showHelp(commandName, true);
            return;
        }

        uint32_t faultId;
        if (!decodeU32(args.at(0), faultId, 31)) {
            interface.writeLine("Invalid fault id specified, expected integer <32.");
            return;
        }

        interface.writeLine("Raising fault " + interface.handle->lookupFaultName(faultId));
        interface.handle->raiseFault(faultId);
    }
};

class AppLowerFaultCommand : public CanmoreCommandHandler<Canmore::DebugClient> {
public:
    AppLowerFaultCommand(): CanmoreCommandHandler("lowerfault") {}

    std::string getArgList() const override { return "[fault id]"; }
    std::string getHelp() const override { return "Lowers the requested fault id."; }

    void callbackSafe(CLIInterface<Canmore::DebugClient> &interface, std::vector<std::string> const &args) override {
        if (args.size() != 1) {
            interface.writeLine("Expected 1 argument");
            interface.showHelp(commandName, true);
            return;
        }

        uint32_t faultId;
        if (!decodeU32(args.at(0), faultId, 31)) {
            interface.writeLine("Invalid fault id specified, expected integer <32.");
            return;
        }

        interface.writeLine("Lowering fault " + interface.handle->lookupFaultName(faultId));
        interface.handle->lowerFault(faultId);
    }
};

class AppUptimeCommand : public CanmoreCommandHandler<Canmore::DebugClient> {
public:
    AppUptimeCommand(): CanmoreCommandHandler("uptime") {}

    std::string getArgList() const override { return ""; }
    std::string getHelp() const override { return "Returns the current uptime."; }

    void callbackSafe(CLIInterface<Canmore::DebugClient> &interface, std::vector<std::string> const &args) override {
        (void) args;
        renderField("Uptime", interface.handle->getUptime().format());
    }
};

class AppVersionCommand : public CanmoreCommandHandler<Canmore::DebugClient> {
public:
    AppVersionCommand(): CanmoreCommandHandler("version") {}

    std::string getArgList() const override { return ""; }
    std::string getHelp() const override { return "Returns the current application version."; }

    void callbackSafe(CLIInterface<Canmore::DebugClient> &interface, std::vector<std::string> const &args) override {
        (void) args;
        renderField("App Version", interface.handle->getVersion());
    }
};

class AppRebootCommand : public CanmoreCommandHandler<Canmore::DebugClient> {
public:
    AppRebootCommand(): CanmoreCommandHandler("reboot") {}

    std::string getArgList() const override { return ""; }
    std::string getHelp() const override { return "Reboots the device."; }

    void callbackSafe(CLIInterface<Canmore::DebugClient> &interface, std::vector<std::string> const &args) override {
        (void) args;
        interface.writeLine("Sending reboot command...");
        interface.handle->reboot();

        // Wait for device to reset
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        interface.close("Closed due to Reboot Request");
    }
};

class AppMemoryStatsCommand : public CanmoreCommandHandler<Canmore::DebugClient> {
public:
    AppMemoryStatsCommand(): CanmoreCommandHandler("memstats") {}

    std::string getArgList() const override { return ""; }
    std::string getHelp() const override { return "Reports device memory statistics."; }

    void callbackSafe(CLIInterface<Canmore::DebugClient> &interface, std::vector<std::string> const &args) override {
        (void) args;
        auto stats = interface.handle->getMemoryStats();
        uint32_t reservedMem = stats.totalMem - stats.heapUse;

        renderField("Memory Usage", formatPercent(stats.arena - stats.keepcost + reservedMem, stats.totalMem), 40);
        renderField("Total memory on chip", formatMemory(stats.totalMem), 40);
        renderField("Static memory reserved", formatMemory(stats.staticUse), 40);
        renderField("Stack memory reserved", formatMemory(stats.stackUse), 40);
        renderField("Total heap memory", formatMemory(stats.heapUse), 40);
        renderField("Heap reserved [Used & Free] (arena)", formatMemory(stats.arena), 40);
        renderField("# of free chunks (ordblks)", std::to_string(stats.ordblks) + " blocks", 40);
        renderField("Total allocated (uordblks)", formatMemory(stats.uordblks), 40);
        renderField("Total free space (fordblks)", formatMemory(stats.fordblks), 40);
        renderField("Free space at top of heap (keepcost)", formatMemory(stats.keepcost), 40);
        if (stats.hblks > 0 || stats.hblkhd > 0) {
            renderField("# of mem-mapped regions (hblks)", std::to_string(stats.hblks) + " blocks", 40);
            renderField("Bytes in mem-mapped regions (hblkhd)", formatMemory(stats.hblkhd), 40);
        }
    }

private:
    std::string formatMemory(unsigned int bytes) {
        std::stringstream output;
        output << std::fixed << std::setprecision(2);
        if (bytes > 1024 * 1024) {
            output << (float) bytes / (1024 * 1024) << " MB";
        }
        else if (bytes > 1024) {
            output << (float) bytes / 1024 << " KB";
        }
        else {
            output << bytes << " bytes";
        }
        return output.str();
    }

    std::string formatPercent(double numerator, double denominator) {
        std::stringstream output;
        output << std::fixed << std::setprecision(2) << 100 * (numerator / denominator) << " %";
        return output.str();
    }
};

class AppMemReadCommand : public CanmoreCommandHandler<Canmore::DebugClient> {
public:
    AppMemReadCommand(): CanmoreCommandHandler("mem_read") {}

    std::string getArgList() const override { return "[address]"; }
    std::string getHelp() const override { return "Reads the specified address in memory"; }

    void callbackSafe(CLIInterface<Canmore::DebugClient> &interface, std::vector<std::string> const &args) override {
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

        if (address % 4 != 0) {
            interface.writeLine("Invalid address: must be 32-bit aligned");
            return;
        }

        uint32_t data = interface.handle->readMemory(address);

        std::stringstream out;
        out << "0x" << std::setw(8) << std::hex << std::uppercase << std::setfill('0') << data;
        interface.writeLine(out.str());
    }
};

class AppMemWriteCommand : public CanmoreCommandHandler<Canmore::DebugClient> {
public:
    AppMemWriteCommand(): CanmoreCommandHandler("mem_write") {}

    std::string getArgList() const override { return "[address] [data]"; }
    std::string getHelp() const override { return "Writes data to the specified address in memory"; }

    void callbackSafe(CLIInterface<Canmore::DebugClient> &interface, std::vector<std::string> const &args) override {
        if (args.size() != 2) {
            interface.writeLine("Expected 2 arguments");
            interface.showHelp(commandName, true);
            return;
        }

        uint32_t address;
        uint32_t data;

        if (!decodeU32(args.at(0), address)) {
            interface.writeLine("Invalid 32-bit integer provided for address");
            return;
        }

        if (address % 4 != 0) {
            interface.writeLine("Invalid address: must be 32-bit aligned");
            return;
        }

        if (!decodeU32(args.at(1), data)) {
            interface.writeLine("Invalid 32-bit integer provided for data");
            return;
        }

        interface.handle->writeMemory(address, data);
    }
};

class AppCanDebugCommand : public CanmoreCommandHandler<Canmore::DebugClient> {
public:
    AppCanDebugCommand(): CanmoreCommandHandler("candbg") {}

    std::string getArgList() const override { return "[action]"; }
    std::string getHelp() const override {
        return "Issues a can debug operation\nValid Actions:\n1: Issue intr en\n2: Issue fifo clear\n3: Reset CAN "
               "hardware";
    }

    void callbackSafe(CLIInterface<Canmore::DebugClient> &interface, std::vector<std::string> const &args) override {
        if (args.size() != 1) {
            interface.writeLine("Expected 1 argument");
            interface.showHelp(commandName, true);
            return;
        }

        std::string const &action = args.at(0);
        if (action == "1") {
            interface.writeLine("Issuing CAN Interrupt Enable Signal");
            interface.handle->canDbgIntrEn();
        }
        else if (action == "2") {
            interface.writeLine("Issuing CAN FIFO Reset Signal");
            interface.handle->canDbgIntrEn();
        }
        else if (action == "3") {
            interface.writeLine("Issuing CAN Debug Reset Subsystem Signal");
            interface.handle->canDbgReset();
        }
        else {
            interface.writeLine("Invalid action: '" + action + "'");
        }
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
    registerCommand(std::make_shared<AppSafetyStatusCommand>());
    registerCommand(std::make_shared<AppMemoryStatsCommand>());
    registerCommand(std::make_shared<AppMemReadCommand>());
    registerCommand(std::make_shared<AppMemWriteCommand>());
    registerCommand(std::make_shared<AppCanDebugCommand>());
    setBackgroundTask(std::make_shared<AppKeepaliveTask>());

    auto devMap = DeviceMap::create();
    uint64_t flashId = handle->getFlashId();
    auto devDescr = devMap.lookupSerial(flashId);

    std::cout << std::endl;
    renderHeader("Connecting to Application");
    renderName(devDescr.name);
    if (devDescr.boardType != "unknown")
        renderField("Board Type", devDescr.boardType);
    else if (flashId != 0)
        renderField("Unique ID", devDescr.hexSerialNum());
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
