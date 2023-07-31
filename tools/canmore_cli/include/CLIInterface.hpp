#pragma once

#include <chrono>
#include <list>
#include <map>
#include <memory>
#include <vector>
#include <sstream>
#include <termios.h>

#include "TerminalDraw.hpp"

template <class T> class CLIInterface;

template <class T>
class CLICommandHandler {
public:
    CLICommandHandler(std::string const& commandName): commandName(commandName){}
    const std::string commandName;

    virtual void callback(CLIInterface<T> &interface, std::vector<std::string> const& args) = 0;

    virtual std::string getArgList() const {return "[???]";}
    virtual std::string getHelp() const {return "No description available";}
};

template <class T>
class CLIBackgroundTask {
public:
    // If this background task returns false, the CLI will exit
    // Useful for checking connection in the background and exiting on lost connection
    virtual void callback(CLIInterface<T> &interface) = 0;
};

class CLICore {
public:
    CLICore(): prompt("> "), promptShown(false), historyItr(cmdHistory.end()) {initTerminal();}
    ~CLICore() {cleanupTerminal();}

    void writeLine(std::string const& line);
    std::string getCommand(std::vector<std::string> &argsOut);  // Note this function may return whenever with an empty string to allow a background task to refresh

    std::string prompt;
    std::string exitMessage;

private:
    bool promptShown;
    std::vector<char> cmdBuffer;
    std::list<std::string> cmdHistory;
    std::list<std::string>::iterator historyItr;
    void initTerminal();
    void cleanupTerminal();

    struct termios oldt;
};

template <class T>
class CLIInterface {
    class HelpCommand: public CLICommandHandler<T> {
    public:
        HelpCommand(): CLICommandHandler<T>("help") {}
        void callback(CLIInterface<T> &interface, std::vector<std::string> const& args) override {interface.showHelp(args.empty() ? "" : args.at(0));}
        std::string getArgList() const override {return "[COMMAND]";}
        std::string getHelp() const override {return "Shows this help message.\nIf COMMAND is provided, only help for that command is printed.";}
    };

    class ClearCommand: public CLICommandHandler<T> {
    public:
        ClearCommand(): CLICommandHandler<T>("clear") {}
        void callback(CLIInterface<T> &interface, std::vector<std::string> const& args) override {(void) args; interface.writeLine(CLEAR_TERMINAL CURSOR_GOTO_START);}
        std::string getArgList() const override {return "";}
        std::string getHelp() const override {return "Clears the terminal.";}
    };

    class ExitCommand: public CLICommandHandler<T> {
    public:
        ExitCommand(): CLICommandHandler<T>("exit") {}
        void callback(CLIInterface<T> &interface, std::vector<std::string> const& args) override {(void) args; interface.close("");}
        std::string getArgList() const override {return "";}
        std::string getHelp() const override {return "Closes the CLI and returns to device selection screen.";}
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
        if (bgTask) bgTask->callback(*this);

        std::vector<std::string> args;
        while (!should_exit) {
            auto cmd = cliCore.getCommand(args);
            if (cmd.size() > 0) {
                auto itr = handlers.find(cmd);
                if (itr == handlers.end())
                    writeLine(cmd + ": command not found");
                else
                    itr->second->callback(*this, args);
                args.clear();
            }

            if (std::chrono::steady_clock::now() - lastRefresh > std::chrono::milliseconds(1500) && !should_exit) {
                if (bgTask) bgTask->callback(*this);
                lastRefresh = std::chrono::steady_clock::now();
            }
        }
    }

    void showHelp(std::string const& cmdName, bool onlyUsage = false) {
        if (cmdName.empty()) {
            for (auto entry : handlers) {
                //writeLine("");
                auto handler = entry.second;
                writeLine(COLOR_NAME + handler->commandName + " " COLOR_BODY + handler->getArgList() + COLOR_RESET);
                writeMultiline(handler->getHelp());
            }
        }
        else {
            auto itr = handlers.find(cmdName);
            if (itr == handlers.end()) {
                writeLine("No such command: " + cmdName);
            }
            else {
                auto handler = itr->second;
                if (onlyUsage) {
                    writeLine("Usage: " + handler->commandName + " " + handler->getArgList());
                } else {
                    writeLine(handler->commandName + " " + handler->getArgList());
                    writeMultiline(handler->getHelp());
                }
            }
        }
    }

    void writeLine(std::string const& line) {
        // Must be used to write lines from background task
        // This ensures that the currently inputted command is preserved while output is entered
        cliCore.writeLine(line);
    }

    void writeMultiline(std::string const& text, std::string const& indent = "\t") {
        if (text.size() > 0) {
            std::istringstream iss(text);
            std::string line;
            while (std::getline(iss, line)) {
                writeLine(line.empty() ? "" : indent + line);
            }
        }
    }

    void close(std::string const& message="") {
        cliCore.exitMessage = message;
        should_exit = true;
    }

    const std::shared_ptr<T> handle;

protected:
    void registerCommand(std::shared_ptr<CLICommandHandler<T>> handler) {
        if (handler->commandName.empty() || handlers.count(handler->commandName) > 0) {
            throw std::runtime_error("Unable to register command: " + handler->commandName);
        }
        handlers.emplace(handler->commandName, handler);
    }

    void setBackgroundTask(std::shared_ptr<CLIBackgroundTask<T>> bgTask) {
        this->bgTask = bgTask;
    }

private:
    bool should_exit;
    CLICore cliCore;
    std::map<std::string, std::shared_ptr<CLICommandHandler<T>>> handlers;
    std::shared_ptr<CLIBackgroundTask<T>> bgTask;
};
