#include "CLIInterface.hpp"

#include <iostream>
#include <sstream>
#include <termios.h>
#include <unistd.h>

void CLICore::initTerminal() {
    // Backup old flags
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
}

void CLICore::cleanupTerminal() {
    if (promptShown) {
        std::cout << std::endl;
    }
    std::cout << exitMessage << std::endl;

    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
}

void CLICore::writeLine(std::string const &line) {
    if (promptShown) {
        std::cout << CURSOR_RETURN << line << CLEAR_LINE_AFTER << std::endl;
        std::cout << prompt;
        for (auto c : cmdBuffer) {
            std::cout << c;
        }
        std::cout << std::flush;
    }
    else {
        std::cout << line << std::endl;
    }
}

std::string CLICore::getCommand(std::vector<std::string> &argsOut) {
    if (!promptShown) {
        std::cout << prompt << std::flush;
        promptShown = true;
    }

    std::string cmdName;

    // Allow early exits for command refreshes
    std::chrono::time_point start = std::chrono::steady_clock::now();
    while (std::chrono::steady_clock::now() - start < std::chrono::milliseconds(1550)) {
        if (!keypressAvailable(1500))
            break;

        int c = getchar();

        // Handle normal characters
        if (c >= 32 && c <= 126) {
            std::cout << (char) c << std::flush;
            cmdBuffer.push_back(c);
        }

        // Handle enter key
        else if (c == '\n') {
            std::cout << std::endl;
            promptShown = false;

            if (cmdBuffer.empty())
                break;

            std::string line(cmdBuffer.begin(), cmdBuffer.end());
            cmdBuffer.clear();

            // History handling
            if (cmdHistory.empty() || cmdHistory.back() != line) {
                if (cmdHistory.size() > 100) {
                    cmdHistory.pop_front();
                }
                cmdHistory.emplace_back(line);
            }
            historyItr = cmdHistory.end();

            // Parse line
            std::istringstream iss(line);
            iss >> cmdName;

            argsOut.clear();
            std::string argToken;
            while (iss >> argToken) {
                argsOut.push_back(argToken);
            }
            break;
        }

        // Check for backspace (according to what the terminal should send)
        else if (c == oldt.c_cc[VERASE]) {
            if (!cmdBuffer.empty()) {
                std::cout << "\b" CLEAR_LINE_AFTER << std::flush;
                cmdBuffer.pop_back();
            }
        }

        else if (c == '\t') {
            // std::cout << "TAB*" << std::flush;
        }

        // Handle escape sequences
        else if (c == '\033') {
            int ec = getchar();
            if (ec == '[') {
                std::string modifiers;
                while (true) {
                    ec = getchar();
                    if (ec == ';' || (ec >= '0' && ec <= '9'))
                        modifiers += (char) ec;
                    else
                        break;
                }

                // Up Arrow
                if (ec == 'A' && modifiers.size() == 0) {
                    if (historyItr != cmdHistory.begin()) {
                        historyItr = std::prev(historyItr);
                        // No need to check if historyIndex is 0, as we've incremented it, so we should never overflow
                        // our bounds
                        cmdBuffer.clear();
                        cmdBuffer.insert(cmdBuffer.begin(), historyItr->begin(), historyItr->end());
                        std::cout << CURSOR_RETURN << prompt << *historyItr << CLEAR_LINE_AFTER << std::flush;
                    }
                }

                // Down arrow
                else if (ec == 'B' && modifiers.size() == 0) {
                    if (historyItr != cmdHistory.end()) {
                        cmdBuffer.clear();
                        std::cout << CURSOR_RETURN << prompt;

                        historyItr = std::next(historyItr);
                        if (historyItr != cmdHistory.end()) {
                            cmdBuffer.insert(cmdBuffer.begin(), historyItr->begin(), historyItr->end());
                            std::cout << *historyItr;
                        }
                        std::cout << CLEAR_LINE_AFTER << std::flush;
                    }
                }
            }
        }

        else if (c == oldt.c_cc[VINTR]) {  // CTRL-C handling
            std::cout << "^C" << std::endl;
            historyItr = cmdHistory.end();
            cmdBuffer.clear();
            std::cout << CURSOR_RETURN << prompt << std::flush;
            break;
        }
    }
    return cmdName;
}

bool decodeU32(const std::string &str, uint32_t &intOut, uint32_t max) {
    const std::string decLookup = "0123456789";
    const std::string hexLookup = "0123456789ABCDEF";

    uint32_t value = 0;
    bool isHex = (str.rfind("0x", 0) == 0);

    // It's safe to assume we've got at least 2 chars in it, since we found 2 characters
    auto searchItr = (isHex ? str.begin() + 2 : str.begin());
    const std::string &lookup = (isHex ? hexLookup : decLookup);
    uint32_t multiplier = (isHex ? 16 : 10);

    // Now search through the string to the end
    while (searchItr != str.end()) {
        // Find index in lookup
        size_t pos = lookup.find(std::toupper(*searchItr++));

        // Fail if its not a valid character
        if (pos == std::string::npos) {
            return false;
        }

        // Fail if it will overflow
        uint32_t maxCurVal = max / multiplier;
        uint32_t maxNextDigit = value % multiplier;

        if (value > maxCurVal) {  // It'll overflow if we add another digit
            return false;
        }
        else if (value == maxCurVal && pos > maxNextDigit) {
            // It'll overflow since we're at the max, and the 1s digit will overflow it
            return false;
        }

        // Perform multiplication
        value *= multiplier;
        value += pos;
    }

    intOut = value;
    return true;
}
