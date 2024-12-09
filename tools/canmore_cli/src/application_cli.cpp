#include "CLIBackends.hpp"
#include "CLIInterface.hpp"
#include "DeviceMap.hpp"
#include "GDBServer.hpp"
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

class AppEnterBootloaderCommand : public CLICommandHandler<Canmore::DebugClient> {
public:
    AppEnterBootloaderCommand(): CLICommandHandler("enterbl") {}

    std::string getArgList() const override { return ""; }
    std::string getHelp() const override { return "Reboots the device into bootloader mode."; }

    void callback(CLIInterface<Canmore::DebugClient> &interface, std::vector<std::string> const &args) override {
        (void) args;
        interface.writeLine("Requesting reboot into bootloader mode...");
        interface.handle->enterBootloader();
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        interface.close("Closed due to Reboot Request");
    }
};

class AppCrashlogCommand : public CLICommandHandler<Canmore::DebugClient> {
public:
    AppCrashlogCommand(): CLICommandHandler("crashlog") {}

    std::string getArgList() const override { return ""; }
    std::string getHelp() const override { return "Returns the full device crash log."; }

    void callback(CLIInterface<Canmore::DebugClient> &interface, std::vector<std::string> const &args) override {
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

class AppSafetyStatusCommand : public CLICommandHandler<Canmore::DebugClient> {
public:
    AppSafetyStatusCommand(): CLICommandHandler("safetystatus") {}

    std::string getArgList() const override { return ""; }
    std::string getHelp() const override { return "Returns the current safety status."; }

    void callback(CLIInterface<Canmore::DebugClient> &interface, std::vector<std::string> const &args) override {
        (void) args;
        auto safetyStatus = interface.handle->getSafetyStatus();
        renderField("Safety Setup", (safetyStatus.safetySetup ? "Yes" : "No"));
        renderField("Safety Inited", (safetyStatus.safetyInitialized ? "Yes" : "No"));
        renderField("Kill State", (safetyStatus.killIsEnabled ? "Enabled" : "Killed"));
        renderField("Error State", (safetyStatus.faultPresent ? COLOR_ERROR "Fault Present" COLOR_RESET : "Normal"));
    }
};

class AppFaultCommand : public CLICommandHandler<Canmore::DebugClient> {
public:
    AppFaultCommand(): CLICommandHandler("fault") {}

    std::string getArgList() const override { return "[action] [fault num (for raise/lower)]"; }
    std::string getHelp() const override {
        return "Interfaces with the Titan Safety Fault System\n"
               "Valid Actions:\n"
               "  list: Lists active faults (Default)\n"
               "  listall: Lists all faults, including inactive ones\n"
               "  raise: Raises the specified fault number\n"
               "  lower: Lowers the specified fault number";
    }

    void callback(CLIInterface<Canmore::DebugClient> &interface, std::vector<std::string> const &args) override {
        if (args.size() > 2) {
            interface.writeLine("Invalid Syntax!");
            interface.showHelp(commandName, true);
            return;
        }

        // Command Parsing
        bool doRaise = false;
        bool doLower = false;
        bool listAll = false;
        if (args.size() > 0) {
            const std::string &action = args.at(0);
            if (action == "raise") {
                doRaise = true;
            }
            else if (action == "lower") {
                doLower = true;
            }
            else if (action == "listall") {
                listAll = true;
            }
            else if (action != "list") {
                interface.writeLine("Invalid Action: " + action);
                return;
            }
        }

        // Do the action
        if (doRaise || doLower) {
            // Doing a raise/lower
            if (args.size() != 2) {
                interface.writeLine("Invalid Syntax: Fault number argument required");
                interface.showHelp(commandName, true);
                return;
            }

            uint32_t faultId;
            if (!decodeU32(args.at(1), faultId, 31)) {
                interface.writeLine("Invalid fault id specified, expected integer <32.");
                interface.writeLine("Use 'listall' action to view ID to name mapping");
                return;
            }

            if (doRaise) {
                interface.writeLine("Raising fault " + interface.handle->lookupFaultName(faultId));
                interface.handle->raiseFault(faultId);
            }
            else {
                interface.writeLine("Lowering fault " + interface.handle->lookupFaultName(faultId));
                interface.handle->lowerFault(faultId);
            }
        }
        else {
            // Doing list, don't need any extra args
            if (args.size() > 1) {
                interface.writeLine("Invalid Syntax!");
                interface.showHelp(commandName, true);
                return;
            }

            // Extract Fault Data
            uint32_t activeFaults = interface.handle->getActiveFaults();
            std::vector<Canmore::FaultData> faultList;
            if (listAll) {
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
                FaultRenderer renderer(faultList, listAll);
                std::string headerRow = renderer.renderHeader();

                // Render header
                renderHeader((listAll ? "All Faults" : "Active Faults"),
                             headerRow.length() - 13);  // Subtract ansi codes
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
            out << " | " << std::setw(multipleWidth) << "Multiple";
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
            out << " | " << std::setw(multipleWidth) << (data.multipleFires ? "Yes" : "No");
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
        const size_t multipleWidth = 8;
        const size_t lineWidth = 6;
        const size_t extraDataWidth = 10;
    };
};

class AppUptimeCommand : public CLICommandHandler<Canmore::DebugClient> {
public:
    AppUptimeCommand(): CLICommandHandler("uptime") {}

    std::string getArgList() const override { return ""; }
    std::string getHelp() const override { return "Returns the current uptime."; }

    void callback(CLIInterface<Canmore::DebugClient> &interface, std::vector<std::string> const &args) override {
        (void) args;
        renderField("Uptime", interface.handle->getUptime().format());
    }
};

class AppVersionCommand : public CLICommandHandler<Canmore::DebugClient> {
public:
    AppVersionCommand(): CLICommandHandler("version") {}

    std::string getArgList() const override { return ""; }
    std::string getHelp() const override { return "Returns the current application version."; }

    void callback(CLIInterface<Canmore::DebugClient> &interface, std::vector<std::string> const &args) override {
        (void) args;
        renderField("App Version", interface.handle->getVersion());
    }
};

class AppRebootCommand : public CLICommandHandler<Canmore::DebugClient> {
public:
    AppRebootCommand(): CLICommandHandler("reboot") {}

    std::string getArgList() const override { return ""; }
    std::string getHelp() const override { return "Reboots the device."; }

    void callback(CLIInterface<Canmore::DebugClient> &interface, std::vector<std::string> const &args) override {
        (void) args;
        interface.writeLine("Sending reboot command...");
        interface.handle->reboot();

        // Wait for device to reset
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        interface.close("Closed due to Reboot Request");
    }
};

class AppGdbServer : public CLICommandHandler<Canmore::DebugClient> {
public:
    AppGdbServer(): CLICommandHandler("gdbserver") {}

    const uint16_t default_port = 3333;

    std::string getArgList() const override { return "[port: Default " + std::to_string(default_port) + "]"; }
    std::string getHelp() const override { return "Starts gdb server over Canmore"; }

    void callback(CLIInterface<Canmore::DebugClient> &interface, std::vector<std::string> const &args) override {
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

class AppMemCommand : public CLICommandHandler<Canmore::DebugClient> {
public:
    AppMemCommand(): CLICommandHandler("mem") {}

    std::string getArgList() const override { return "[action] [...]"; }
    std::string getHelp() const override {
        return "Performs various low-level actions to device memory:\n"
               "Actions:\n"
               "  read [address]\n"
               "    Reads the word at the specified address\n"
               "  write [address] [data]\n"
               "    Writes the provided data word to the specified word-aligned address\n"
               "  readstr [address] [optional max len]\n"
               "    Reads the null-terminated string at the specified word-aligned address\n"
               "    The max len determines how far to search for the null termination (Defaults to 256)\n"
               "  dump [address] [length]\n"
               "    Shows a hex dump starting at the provided address (must be word-aligned)";
    }

    void callback(CLIInterface<Canmore::DebugClient> &interface, std::vector<std::string> const &args) override {
        if (args.size() < 2 || args.size() > 3) {
            interface.writeLine("Invalid Syntax: Expected 2-3 arguments");
            interface.showHelp(commandName, true);
            return;
        }

        // Decode Action
        const std::string &action = args.at(0);
        bool isWrite = false;
        bool readString = false;
        bool isDump = false;
        if (action == "write") {
            isWrite = true;
        }
        else if (action == "dump") {
            isDump = true;
        }
        else if (action == "readstr") {
            readString = true;
        }
        else if (action != "read") {
            interface.writeLine("Invalid Action: " + action);
            interface.showHelp(commandName, true);
            return;
        }

        // Decode Address
        uint32_t address;
        if (!decodeU32(args.at(1), address)) {
            interface.writeLine("Invalid 32-bit integer provided for address");
            return;
        }

        if (address % 4 != 0 && !readString) {
            interface.writeLine("Invalid address: must be 32-bit aligned");
            return;
        }

        // Decode optional third argument
        bool hasThirdArg = (args.size() >= 3);
        uint32_t thirdArg;
        if (hasThirdArg) {
            if (!decodeU32(args.at(2), thirdArg)) {
                interface.writeLine("Invalid integer provided in third argument (must be either hex or decimal)");
                return;
            }
        }

        // Perform operation
        if (isWrite) {
            // Perform Write
            if (!hasThirdArg) {
                interface.writeLine("Invalid Syntax: Write requires data argument to be provided");
                interface.showHelp(commandName, true);
                return;
            }

            interface.handle->writeMemory(address, thirdArg);
        }
        else if (readString) {
            uint32_t maxLen = (hasThirdArg ? thirdArg : 256);

            // Perform String Read
            uint32_t curLen = 0;
            std::stringstream data;
            data << COLOR_HEADER "String @0x" << std::hex << std::setw(8) << std::setfill('0') << address
                 << ": \"" COLOR_RESET;

            bool addrValid = false;
            uint32_t addrAligned = 0;
            uint32_t curData = 0;
            while (curLen < maxLen) {
                uint32_t curIdx = (uint32_t) ((address + curLen) % 4);
                uint32_t curWord = (uint32_t) ((address + curLen) - curIdx);

                if (!addrValid || curWord != addrAligned) {
                    addrAligned = curWord;
                    curData = interface.handle->readMemory(addrAligned);
                }

                char curChar = (curData >> (curIdx * 8)) & 0xFF;
                if (curChar == 0) {
                    data << COLOR_HEADER "\"" COLOR_RESET;
                    interface.writeLine(data.str());
                    return;
                }
                data << curChar;
                curLen++;
            }
            interface.writeLine(data.str());
            interface.writeLine(COLOR_NOTICE "[WARN] Reached maximum length of " + std::to_string(maxLen) +
                                COLOR_RESET);
        }
        else if (isDump) {
            // Perform hex dump
            if (!hasThirdArg) {
                interface.writeLine("Invalid Syntax: Hex dump requires length argument to be provided");
                interface.showHelp(commandName, true);
                return;
            }

            uint32_t length = thirdArg;

            if (length % 4 != 0 || length == 0) {
                interface.writeLine("Invalid Length: Must be divisible by word size");
                return;
            }

            renderHeader("Memory Hex Dump");
            std::vector<uint8_t> data;
            data.reserve(length);

            for (uint32_t i = 0; i < length; i += 4) {
                uint32_t word = interface.handle->readMemory(address + i);
                data.push_back(word & 0xFF);
                data.push_back((word >> 8) & 0xFF);
                data.push_back((word >> 16) & 0xFF);
                data.push_back((word >> 24) & 0xFF);
            }

            DumpHex(address, data.data(), data.size());
        }
        else {
            if (hasThirdArg) {
                interface.writeLine("Invalid Syntax: Expected 2 arguments");
                interface.showHelp(commandName, true);
                return;
            }

            // Perform Word Read
            uint32_t data = interface.handle->readMemory(address);

            std::stringstream out;
            out << "0x" << std::setw(8) << std::hex << std::uppercase << std::setfill('0') << data;
            interface.writeLine(out.str());
        }
    }
};

class AppRemoteCmdPrefix : public CLICommandPrefixHandler<Canmore::DebugClient> {
public:
    AppRemoteCmdPrefix(): CLICommandPrefixHandler('@') {}

    void callback(CLIInterface<Canmore::DebugClient> &interface, std::vector<std::string> const &args) override {
        if (args.size() == 0) {
            interface.writeLine("Cannot send empty remote command");
            return;
        }
        std::string response;
        int returncode = interface.handle->executeRemoteCmd(args, response);

        interface.writeMultiline(response, "");
        if (response.length() == CANMORE_DBG_REMOTE_CMD_RESP_MAX_LEN - 1) {
            interface.writeLine(COLOR_NOTICE "... Notice: Output may be truncated");
        }
        if (returncode != 0) {
            interface.writeLine(COLOR_NOTICE "Client responded with non-zero status code: " +
                                std::to_string(returncode));
        }
    }

    void showHelp(CLIInterface<Canmore::DebugClient> &interface, bool showUsage) override {
        if (showUsage) {
            std::stringstream oss;
            oss << "Usage: " << prefixChar << "[Remote Command] [Remote Command Args]";
            interface.writeLine(oss.str());
            return;
        }

        try {
            // Read remote help data from device if it hasn't been fetched yet
            if (!remoteHelpFetched) {
                // Get the command count from the client
                std::string response;
                std::vector<std::string> cmdCountCmd = { "help", "-n" };
                int returncode = interface.handle->executeRemoteCmd(cmdCountCmd, response);
                if (returncode != 0) {
                    throw Canmore::DebugError("Non-zero status code when retreiving command count");
                }

                // Decode string to a number
                uint32_t cmdCount;
                if (!decodeU32(response, cmdCount)) {
                    throw Canmore::DebugError("Invalid integer response from command count: " + response);
                }

                // Retrieve the help from the device, formatting in the same format as our help
                std::stringstream remoteHelp;
                if (cmdCount > 0) {
                    remoteHelp << std::endl
                               << COLOR_TITLE "====Remote Commands (Prefixed with " << prefixChar << ")====" COLOR_RESET
                               << std::endl;
                }

                for (uint32_t i = 0; i < cmdCount; i++) {
                    std::vector<std::string> getCmdHelpArgs = { "help", "-i", std::to_string(i) };
                    int returncode = interface.handle->executeRemoteCmd(getCmdHelpArgs, response);
                    if (returncode != 0) {
                        throw Canmore::DebugError("Non-zero status code when retreiving remote command help");
                    }

                    std::istringstream iss(response);

                    // Retrieve the command name line
                    std::string line;
                    if (!std::getline(iss, line)) {
                        throw Canmore::DebugError("Malformed response - Could not retrieve command name from help");
                    }
                    remoteHelp << COLOR_NAME << prefixChar << line;

                    // Retrieve the command usage line
                    if (!std::getline(iss, line)) {
                        throw Canmore::DebugError("Malformed response - Could not retrieve command usage from help");
                    }
                    remoteHelp << " " COLOR_BODY << line << COLOR_RESET << std::endl;

                    // Consume the rest of the response as help message
                    bool had_help = false;
                    while (std::getline(iss, line)) {
                        had_help = true;
                        if (!line.empty()) {
                            remoteHelp << "\t" << line << std::endl;
                        }
                        else {
                            remoteHelp << std::endl;
                        }
                    }
                    if (!had_help) {
                        remoteHelp << "\t<No Help Message Provided>" << std::endl;
                    }
                }
                remoteHelpMsg = remoteHelp.str();
                remoteHelpFetched = true;
            }

            // Show the remote help message
            interface.writeLine(remoteHelpMsg);
        } catch (Canmore::CanmoreError &e) {
            interface.writeLine(COLOR_ERROR "Exception caught while fetching remote help:" COLOR_RESET);
            interface.writeLine(COLOR_ERROR "  what(): " + std::string(e.what()) + COLOR_RESET);
        }
    }

private:
    bool remoteHelpFetched = false;
    std::string remoteHelpMsg;
};

ApplicationCLI::ApplicationCLI(std::shared_ptr<Canmore::DebugClient> handle): CLIInterface(handle) {
    registerCommand(std::make_shared<AppUptimeCommand>());
    registerCommand(std::make_shared<AppVersionCommand>());
    registerCommand(std::make_shared<AppRebootCommand>());
    registerCommand(std::make_shared<AppEnterBootloaderCommand>());
    registerCommand(std::make_shared<AppCrashlogCommand>());
    registerCommand(std::make_shared<AppFaultCommand>());
    registerCommand(std::make_shared<AppSafetyStatusCommand>());
    registerCommand(std::make_shared<AppMemCommand>());
    registerCommand(std::make_shared<AppGdbServer>());
    registerCommandPrefix(std::make_shared<AppRemoteCmdPrefix>());
    setBackgroundTask(std::make_shared<AppKeepaliveTask>());

    if (!quietConnect) {
        auto devMap = DeviceMap::create();
        uint64_t flashId = handle->getFlashId();
        auto devDescr = devMap.lookupSerial(flashId);

        // Show general device information on connect

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
}
