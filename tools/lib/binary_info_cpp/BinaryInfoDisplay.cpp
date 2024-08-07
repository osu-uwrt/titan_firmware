#include "BinaryInfo.hpp"

#include <cstring>
#include <iomanip>
#include <iostream>
#include <sstream>

#define COLOR_TITLE "\033[1;35m"
#define COLOR_ID "\033[0m"
#define COLOR_HEADER "\033[1;90m"
#define COLOR_NAME "\033[0;33m"
#define COLOR_BODY "\033[0;32m"
#define COLOR_RESET "\033[0m"

static void printRow(const char *name, const char *value, bool indent = false) {
    // Don't print empty field
    if (*value == 0) {
        return;
    }

    if (indent)
        std::cout << "\t";
    std::cout << COLOR_HEADER << name << (strlen(name) < 7 ? ":\t" : ":");
    if (!indent)
        std::cout << "\t";
    std::cout << COLOR_NAME << value << COLOR_RESET << std::endl;
}

static void printRow(const char *name, std::vector<std::string> values, bool indent = false) {
    // Don't print empty field
    if (values.size() == 0) {
        return;
    }
    if (values.size() == 1) {
        printRow(name, values.at(0).c_str(), indent);
        return;
    }

    if (indent)
        std::cout << "\t";
    std::cout << COLOR_HEADER << name << ":" COLOR_RESET << std::endl;
    for (auto &value : values) {
        std::cout << COLOR_NAME "\t\t" << value << COLOR_RESET << std::endl;
    }
}

static void printRow(const char *name, std::map<uint, std::vector<std::string>> values) {
    if (values.size() == 0) {
        return;
    }

    std::cout << COLOR_HEADER << name << ":" COLOR_RESET << std::endl;
    for (auto &subval : values) {
        auto str = std::to_string(subval.first);
        printRow(str.c_str(), subval.second, true);
    }
}

static void printRow(const char *name, int value) {
    std::stringstream ss;
    ss << "0x" << std::hex << std::setw(8) << std::setfill('0') << value;
    auto str = ss.str();
    printRow(name, str.c_str());
}

namespace BinaryInfo {

void dumpAppInfo(AppInfo &app) {
    // Primary Information
    printRow("Name", app.programName.c_str());
    printRow("Description", app.programDescription.c_str());
    printRow("Version", app.programVersion.c_str());
    printRow("Board Name", app.boardType.c_str());

    // Device Addressing
    printRow("Device IP Addr", app.deviceIpAddress.c_str());
    printRow("Agent IP Addr", app.agentIpAddress.c_str());
    if (app.agentPort != 0) {
        auto s = std::to_string(app.agentPort);
        printRow("Agent Port", s.c_str());
    }

    if (app.clientIds.size() == 1) {
        auto s = std::to_string(app.clientIds.at(0));
        printRow("Client ID", s.c_str());
    }
    else if (app.clientIds.size() > 1) {
        std::cout << COLOR_HEADER << "Client ID List:" COLOR_RESET << std::endl;
        for (auto clientId : app.clientIds) {
            std::cout << COLOR_NAME "\t\t" << clientId << COLOR_RESET << std::endl;
        }
    }

    // Extra Information
    printRow("Build Date", app.programBuildDate.c_str());
    printRow("URL", app.programUrl.c_str());
    printRow("SDK Version", app.sdkVersion.c_str());
    printRow("Boot 2 Name", app.boot2Name.c_str());

    printRow("Binary Start", app.binaryStart);
    if (app.binaryEnd != 0) {
        printRow("Binary End", app.binaryEnd);
    }
    if (app.blAppBase != 0) {
        printRow("Next App Addr", app.blAppBase);
    }

    printRow("Build Attrs", app.buildAttributes);
    printRow("Features", app.programFeatures);
    printRow("Pins", app.pins);

    if (!app.message.empty()) {
        printRow("Message", app.message.c_str());
    }
}

};  // namespace BinaryInfo
