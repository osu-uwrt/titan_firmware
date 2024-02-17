#include "DFCDaemon.hpp"

#include <string.h>

#define bind_reg_cb(func) std::bind(func, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3)

// TODO: Update to actual version
const char version_str[] = "<No Version Assigned>";

CanmoreLinuxServer::CanmoreLinuxServer(int ifIndex, uint8_t clientId):
    Canmore::RegMappedCANServer(ifIndex, clientId, CANMORE_TITAN_CHAN_CONTROL_INTERFACE,
                                CANMORE_TITAN_CONTROL_INTERFACE_MODE_LINUX) {
    // Define general control page
    auto gen_control = Canmore::RegMappedRegisterPage::create();
    gen_control->addConstRegister(CANMORE_LINUX_GEN_CONTROL_MAGIC_OFFSET, CANMORE_LINUX_GEN_CONTROL_MAGIC_VALUE);
    gen_control->addConstRegister(CANMORE_LINUX_GEN_CONTROL_LOWER_FLASH_ID, 0xEFBEADDE);
    gen_control->addConstRegister(CANMORE_LINUX_GEN_CONTROL_UPPER_FLASH_ID, 0xCEFAEDFE);

    gen_control->addCallbackRegister(CANMORE_LINUX_GEN_CONTROL_RESTART_DAEMON_OFFSET, REGISTER_PERM_WRITE_ONLY,
                                     bind_reg_cb(&CanmoreLinuxServer::restartDaemonCb));

    addRegisterPage(CANMORE_LINUX_GEN_CONTROL_PAGE_NUM, std::move(gen_control));

    addByteMappedPage(CANMORE_LINUX_VERSION_STRING_PAGE_NUM, REGISTER_PERM_READ_ONLY,
                      std::span<uint8_t>((uint8_t *) version_str, sizeof(version_str)));

    // Define TTY Control Page
    auto tty_control = Canmore::RegMappedRegisterPage::create();
    tty_control->addCallbackRegister(CANMORE_LINUX_TTY_CONTROL_ENABLE_OFFSET, REGISTER_PERM_READ_WRITE,
                                     bind_reg_cb(&CanmoreLinuxServer::enableTtyCb));
    tty_control->addMemoryRegister(CANMORE_LINUX_TTY_CONTROL_WINDOW_SIZE_OFFSET, REGISTER_PERM_READ_WRITE,
                                   &windowSzReg_);
    addRegisterPage(CANMORE_LINUX_TTY_CONTROL_PAGE_NUM, std::move(tty_control));

    // Define terminal environment variable string
    termStrBuf_.resize(REG_MAPPED_PAGE_SIZE);
    addByteMappedPage(CANMORE_LINUX_TTY_TERMINAL_PAGE_NUM, REGISTER_PERM_READ_WRITE, termStrBuf_);
}

bool CanmoreLinuxServer::restartDaemonCb(uint16_t addr, bool is_write, uint32_t *data_ptr) {
    if (*data_ptr == CANMORE_LINUX_GEN_CONTROL_RESTART_DAEMON_MAGIC) {
        shouldStop_ = true;
        return true;
    }
    else {
        return false;
    }
}

void CanmoreLinuxServer::getTtyInitialConfig(std::string &termEnv, uint16_t &initialRows, uint16_t &initialCols) {
    // Decode window size register
    initialRows = windowSzReg_ >> 16;
    initialCols = (uint16_t) (windowSzReg_ & 0xFFFF);

    // Decode the terminal environment string
    const char *termEnvPtr = reinterpret_cast<char *>(termStrBuf_.data());
    size_t termEnvSize = strnlen(termEnvPtr, termStrBuf_.size());
    termEnv.assign(termEnvPtr, termEnvSize);
}

bool CanmoreLinuxServer::enableTtyCb(uint16_t addr, bool is_write, uint32_t *data_ptr) {
    if (is_write) {
        if (*data_ptr == 1) {
            remoteTtyEnabled_ = true;
            return true;
        }
        else if (*data_ptr == 0) {
            remoteTtyEnabled_ = false;
            return true;
        }
        else {
            return false;
        }
    }
    else {
        *data_ptr = remoteTtyEnabled_;
        return true;
    }
}
