#include "DFCDaemon.hpp"

#include <fstream>
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

    // Define command string
    cmdBuf_.resize(REG_MAPPED_PAGE_SIZE);
    addByteMappedPage(CANMORE_LINUX_TTY_CMD_PAGE_NUM, REGISTER_PERM_READ_WRITE, cmdBuf_);

    // Define File Buffer page
    fileBuf_.resize(REG_MAPPED_PAGE_SIZE);
    addByteMappedPage(CANMORE_LINUX_FILE_BUFFER_PAGE_NUM, REGISTER_PERM_READ_WRITE, fileBuf_);

    // Define File Upload Control page
    auto upload_control = Canmore::RegMappedRegisterPage::create();
    upload_control->addMemoryRegister(CANMORE_LINUX_UPLOAD_CONTROL_FILENAME_LENGTH_OFFSET, REGISTER_PERM_WRITE_ONLY,
                                      &filenameLengthReg_);
    upload_control->addMemoryRegister(CANMORE_LINUX_UPLOAD_CONTROL_DATA_LENGTH_OFFSET, REGISTER_PERM_WRITE_ONLY,
                                      &dataLengthReg_);
    upload_control->addMemoryRegister(CANMORE_LINUX_UPLOAD_CONTROL_CRC_OFFSET, REGISTER_PERM_WRITE_ONLY, &crc32Reg_);
    upload_control->addMemoryRegister(CANMORE_LINUX_UPLOAD_CONTROL_CLEAR_FILE_OFFSET, REGISTER_PERM_WRITE_ONLY,
                                      &clearFileReg_);
    upload_control->addMemoryRegister(CANMORE_LINUX_UPLOAD_CONTROL_FILE_INODE_OFFSET, REGISTER_PERM_WRITE_ONLY,
                                      &fileINodeReg_);
    upload_control->addCallbackRegister(CANMORE_LINUX_UPLOAD_CONTROL_WRITE_OFFSET, REGISTER_PERM_WRITE_ONLY,
                                        bind_reg_cb(&CanmoreLinuxServer::triggerWriteBufToFile));
    upload_control->addMemoryRegister(CANMORE_LINUX_UPLOAD_CONTROL_WRITE_STATUS_OFFSET, REGISTER_PERM_READ_ONLY,
                                      &writeStatusReg_);
    addRegisterPage(CANMORE_LINUX_UPLOAD_CONTROL_PAGE_NUM, std::move(upload_control));
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

void CanmoreLinuxServer::getTtyInitialConfig(std::string &termEnv, uint16_t &initialRows, uint16_t &initialCols,
                                             std::string &cmd) {
    // Decode window size register
    initialRows = windowSzReg_ >> 16;
    initialCols = (uint16_t) (windowSzReg_ & 0xFFFF);

    // Decode the terminal environment string
    const char *termEnvPtr = reinterpret_cast<char *>(termStrBuf_.data());
    size_t termEnvSize = strnlen(termEnvPtr, termStrBuf_.size());
    termEnv.assign(termEnvPtr, termEnvSize);

    // Decode the terminal command string
    const char *cmdPtr = reinterpret_cast<char *>(cmdBuf_.data());
    size_t cmdSize = strnlen(cmdPtr, cmdBuf_.size());
    cmd.assign(cmdPtr, cmdSize);
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

bool CanmoreLinuxServer::triggerWriteBufToFile(uint16_t addr, bool is_write, uint32_t *data_ptr) {
    bool wantWrite = *data_ptr != 0;
    if (!lastWrittenWrite_ && wantWrite) {
        // read filename out of buffer page
        char filename[(const uint32_t) filenameLengthReg_ + 1] = { 0 };
        for (int i = 0; i < filenameLengthReg_; i++) {
            filename[i] = (char) fileBuf_.at(i);
        }

        // open file
        std::ofstream file;
        file.open(filename);

        // do write
        for (uint32_t i = 0; i < dataLengthReg_; i++) {
            char chunk_int = (char) fileBuf_.at(i + filenameLengthReg_);
            file.write(&chunk_int, sizeof(chunk_int));
        }

        file.close();
    }

    lastWrittenWrite_ = wantWrite;
    return true;  // TODO: add error handling
}

void CanmoreLinuxServer::forceTTYdisconnect() noexcept {
    // Sends the tty disconnect (should be sent on startup) to kick any canmmore cli instances out of the remote sh mode
    canid_t can_id = CAN_EFF_FLAG | CANMORE_REMOTE_TTY_CALC_ID_C2A(clientId, CANMORE_REMOTE_TTY_SUBCH_CONTROL,
                                                                   CANMORE_REMOTE_TTY_CMD_DISCONNECT_ID);

    canmore_remote_tty_cmd_disconnect_t pkt = { .pkt = { .is_err = true } };
    transmitFrameNoexcept(can_id, pkt.data, sizeof(pkt));
}
