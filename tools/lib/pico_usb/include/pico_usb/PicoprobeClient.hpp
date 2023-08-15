#pragma once

#include "BinaryInfo.hpp"
#include "RP2040FlashInterface.hpp"
#include "pico_usb/flash_getid.h"

#include <map>
#include <string>

namespace PicoUSB {

class PicoprobeError : public std::runtime_error {
public:
    PicoprobeError(const char *msg): std::runtime_error(msg) {};
    PicoprobeError(const std::string &msg): std::runtime_error(msg) {};
};

class OpenOCDInstance {
public:
    struct OpenOCDVersion {
        OpenOCDVersion(): valid(false) {}
        OpenOCDVersion(uint8_t major, uint8_t minor): valid(true), major(major), minor(minor) {}
        std::string str() { return "v" + std::to_string(major) + "." + std::to_string(minor); }

        bool valid;
        uint8_t major;
        uint8_t minor;
    };

    OpenOCDInstance();
    ~OpenOCDInstance();

    std::string sendCommand(std::string cmd, int timeout_ms = 5000);
    void init();
    OpenOCDVersion getVersion();

    const bool ranCustomProbeInit;
    const bool enableStderr;

private:
    void writeData(std::string data);
    std::string readData(int timeout_ms);
    pid_t openocdPid;
    int stdinFd;
    int stdoutFd;
};

class RP2040OCDTarget {
public:
    RP2040OCDTarget(std::string probeSerial);

    void reboot() {
        openocd->sendCommand("reset");
        openocd.reset();
    }
    uint32_t lookupRomFunc(char c1, char c2) { return funcTable.at((c2 << 8) | c1); }
    uint32_t callFunction(uint32_t addr, uint32_t r0 = 0, uint32_t r1 = 0, uint32_t r2 = 0, uint32_t r3 = 0,
                          int timeout_ms = 5000);

    uint32_t readWord(uint32_t addr, int width = 32);
    std::vector<uint32_t> readMemory(uint32_t addr, uint32_t count, int width = 32);
    void writeWord(uint32_t addr, uint32_t data, int width = 32);
    void writeMemory(uint32_t addr, std::vector<uint32_t> data, int width = 32);

    int bootromVersion;

private:
    static bool showedVersionWarning;

    std::string probeSerial;
    std::unique_ptr<OpenOCDInstance> openocd;

    void initCommon();
    void initRescue();
    void initNormal();
    void readoutRom();
    std::map<uint16_t, uint16_t> funcTable;
};

class PicoprobeClient : public RP2040FlashInterface {
public:
    enum FlashState { FlashUnknown = 0, FlashXIP, FlashCommand };

    PicoprobeClient(std::string serialStr);

    void eraseSector(uint32_t addr);
    void readBytes(uint32_t addr, std::array<uint8_t, UF2_PAGE_SIZE> &bytesOut);
    void writeBytes(uint32_t addr, std::array<uint8_t, UF2_PAGE_SIZE> &bytes);
    bool verifyBytes(uint32_t addr, std::array<uint8_t, UF2_PAGE_SIZE> &bytes);
    void reboot();

    uint32_t tryGetBootloaderSize() { return firstApp.isBootloader ? firstApp.blAppBase - FLASH_BASE : 0; }
    uint64_t getFlashId() { return cachedFlashId; }
    uint32_t getFlashSize() { return cachedFlashSize; }
    bool shouldWarnOnBootloaderOverwrite() { return false; }

private:
    RP2040OCDTarget target;
    FlashState flashState;
    uint64_t cachedFlashId;
    uint32_t cachedFlashSize;
    std::vector<uint32_t> writeBuffer;
    uint32_t writeBufferAddr;

    void flushWriteBuffer();
    void enterXIPMode();
    void enterCommandMode();
    void notifyCommandDone();  // Must be called after running flash commands requiring enterCommandMode()

    flash_info_t readFlashInfo();

public:
    BinaryInfo::AppInfo firstApp, nestedApp;
};

class PicoprobeDevice : public RP2040Device {
public:
    PicoprobeDevice(std::string serialStr): serialStr(serialStr), itf(new PicoprobeClient(serialStr)) {}

    uint64_t getFlashId() override { return itf->getFlashId(); }
    std::string getInterface() const override { return "Picoprobe (SWD)"; }
    bool supportsFlashInterface() const override { return true; }

    void getAdditionalInfo(std::vector<std::pair<std::string, std::string>> &infoOut) override {
        BinaryInfo::reportVersionInfo(infoOut, itf->firstApp, itf->nestedApp);
    }

    std::shared_ptr<RP2040FlashInterface> getFlashInterface() override { return itf; }

private:
    std::string serialStr;
    std::shared_ptr<PicoprobeClient> itf;
};

};  // namespace PicoUSB
