#pragma once

#include "TerminalDraw.hpp"
#include "canmore_cpp/Canmore.hpp"

#include <chrono>
#include <list>
#include <map>
#include <memory>
#include <sstream>
#include <termios.h>
#include <vector>

bool decodeU32(const std::string &str, uint32_t &intOut, uint32_t max = UINT32_MAX);
void DumpHex(uint32_t address, const void *data, size_t size);

template <class T> class CLIInterface;

template <class T> class CLICommandHandler {
public:
    CLICommandHandler(std::string const &commandName): commandName(commandName) {}
    const std::string commandName;

    virtual void callback(CLIInterface<T> &interface, std::vector<std::string> const &args) = 0;

    virtual std::string getArgList() const { return "[???]"; }
    virtual std::string getHelp() const { return "No description available"; }
};

template <class T> class CLICommandPrefixHandler {
public:
    CLICommandPrefixHandler(char prefixChar): prefixChar(prefixChar) {}
    const char prefixChar;

    virtual void callback(CLIInterface<T> &interface, std::vector<std::string> const &args) = 0;
    virtual void showHelp(CLIInterface<T> &interface, bool onlyUsage = false) = 0;
};

template <class T> class CLIBackgroundTask {
public:
    // If this background task returns false, the CLI will exit
    // Useful for checking connection in the background and exiting on lost connection
    virtual void callback(CLIInterface<T> &interface) = 0;
};

class CLICore {
public:
    CLICore(): prompt("> "), promptShown(false), historyItr(cmdHistory.end()) { initTerminal(); }
    ~CLICore() { cleanupTerminal(); }

    void writeLine(std::string const &line);
    std::string getCommand(std::vector<std::string> &argsOut);  // Note this function may return whenever with an empty
                                                                // string to allow a background task to refresh

    void tempRestoreTerm();
    void tempReinitTerm();

    std::string prompt;
    std::string exitMessage;

private:
    bool promptShown;
    std::vector<char> cmdBuffer;
    std::list<std::string> cmdHistory;
    std::list<std::string>::iterator historyItr;
    void initTerminal();
    void cleanupTerminal() noexcept;

    struct termios oldt;
    struct termios newt;
};

template <class T> class CLIInterface {
    class HelpCommand : public CLICommandHandler<T> {
    public:
        HelpCommand(): CLICommandHandler<T>("help") {}
        void callback(CLIInterface<T> &interface, std::vector<std::string> const &args) override {
            interface.showHelp(args.empty() ? "" : args.at(0));
        }
        std::string getArgList() const override { return "[COMMAND]"; }
        std::string getHelp() const override {
            return "Shows this help message.\nIf COMMAND is provided, only help for that command is printed.";
        }
    };

    class ClearCommand : public CLICommandHandler<T> {
    public:
        ClearCommand(): CLICommandHandler<T>("clear") {}
        void callback(CLIInterface<T> &interface, std::vector<std::string> const &args) override {
            (void) args;
            interface.writeLine(CLEAR_TERMINAL CURSOR_GOTO_START);
        }
        std::string getArgList() const override { return ""; }
        std::string getHelp() const override { return "Clears the terminal."; }
    };

    class ExitCommand : public CLICommandHandler<T> {
    public:
        ExitCommand(): CLICommandHandler<T>("exit") {}
        void callback(CLIInterface<T> &interface, std::vector<std::string> const &args) override {
            (void) args;
            interface.close("");
        }
        std::string getArgList() const override { return ""; }
        std::string getHelp() const override { return "Closes the CLI and returns to device selection screen."; }
    };

public:
    CLIInterface(std::shared_ptr<T> handle): handle(handle) {
        registerCommand(std::make_shared<ClearCommand>());
        registerCommand(std::make_shared<ExitCommand>());
        registerCommand(std::make_shared<HelpCommand>());
    }

    virtual std::string getCliName() const = 0;

    void run() {
        writeLine("Connected to " + getCliName() + " CLI. Type 'help' for a list of commands.");
        cliCore.prompt = COLOR_PROMPT + getCliName() + "> " COLOR_RESET;
        cliCore.exitMessage = "Connection Closed";

        // Background task refreshing
        std::chrono::time_point lastRefresh = std::chrono::steady_clock::now();
        if (bgTask)
            bgTask->callback(*this);

        std::vector<std::string> args;
        should_exit = false;
        while (!should_exit) {
            auto cmd = cliCore.getCommand(args);
            if (cmd.size() > 0) {
                try {
                    // First try to find prefix command
                    auto prefixItr = prefixHandlers.find(cmd.at(0));
                    if (prefixItr != prefixHandlers.end()) {
                        if (cmd.size() > 1) {
                            // Add the remainder of the prefix command to the args
                            args.insert(args.begin(), cmd.substr(1));
                        }
                        // Execute the command
                        prefixItr->second->callback(*this, args);
                    }
                    else {
                        // Not a special prefix, try to find a normal command
                        auto itr = handlers.find(cmd);
                        if (itr == handlers.end())
                            writeLine(cmd + ": command not found");
                        else
                            itr->second->callback(*this, args);
                    }
                    args.clear();
                } catch (Canmore::CanmoreError &e) {
                    writeLine(COLOR_ERROR "Exception caught while running command:" COLOR_RESET);
                    writeLine(COLOR_ERROR "  what(): " + std::string(e.what()) + COLOR_RESET);
                }
            }

            if (std::chrono::steady_clock::now() - lastRefresh > std::chrono::milliseconds(1500) && !should_exit) {
                if (bgTask)
                    bgTask->callback(*this);
                lastRefresh = std::chrono::steady_clock::now();
            }
        }
    }

    void showHelp(std::string const &cmdName, bool onlyUsage = false) {
        if (cmdName.empty()) {
            for (const auto &entry : handlers) {
                auto handler = entry.second;
                writeLine(COLOR_NAME + handler->commandName + " " COLOR_BODY + handler->getArgList() + COLOR_RESET);
                writeMultiline(handler->getHelp());
            }

            for (const auto &entry : prefixHandlers) {
                entry.second->showHelp(*this);
            }
        }
        else {
            if (cmdName.size() == 1) {
                auto prefixItr = prefixHandlers.find(cmdName.at(0));
                if (prefixItr != prefixHandlers.end()) {
                    prefixItr->second->showHelp(*this, onlyUsage);
                    return;
                }
            }

            auto itr = handlers.find(cmdName);
            if (itr == handlers.end()) {
                writeLine("No such command: " + cmdName);
            }
            else {
                auto handler = itr->second;
                if (onlyUsage) {
                    writeLine("Usage: " + handler->commandName + " " + handler->getArgList());
                }
                else {
                    writeLine(handler->commandName + " " + handler->getArgList());
                    writeMultiline(handler->getHelp());
                }
            }
        }
    }

    void writeLine(std::string const &line) {
        // Must be used to write lines from background task
        // This ensures that the currently inputted command is preserved while output is entered
        cliCore.writeLine(line);
    }

    void writeMultiline(std::string const &text, std::string const &indent = "\t") {
        if (text.size() > 0) {
            std::istringstream iss(text);
            std::string line;
            while (std::getline(iss, line)) {
                writeLine(line.empty() ? "" : indent + line);
            }
        }
    }

    void close(std::string const &message = "") {
        cliCore.exitMessage = message;
        should_exit = true;
    }

    void tempRestoreTerm() { cliCore.tempRestoreTerm(); }
    void tempReinitTerm() { cliCore.tempReinitTerm(); }

    const std::shared_ptr<T> handle;

protected:
    void registerCommand(std::shared_ptr<CLICommandHandler<T>> handler) {
        if (handler->commandName.empty() || handlers.count(handler->commandName) > 0) {
            throw std::runtime_error("Unable to register command: " + handler->commandName);
        }
        handlers.emplace(handler->commandName, handler);
    }

    void registerCommandPrefix(std::shared_ptr<CLICommandPrefixHandler<T>> handler) {
        if (prefixHandlers.count(handler->prefixChar) > 0) {
            throw std::runtime_error("Prefix already registered");
        }
        prefixHandlers.emplace(handler->prefixChar, handler);
    }

    void setBackgroundTask(std::shared_ptr<CLIBackgroundTask<T>> bgTask) { this->bgTask = bgTask; }

private:
    bool should_exit;
    CLICore cliCore;
    std::map<std::string, std::shared_ptr<CLICommandHandler<T>>> handlers;
    std::map<char, std::shared_ptr<CLICommandPrefixHandler<T>>> prefixHandlers;
    std::shared_ptr<CLIBackgroundTask<T>> bgTask;
};
