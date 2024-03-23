#include "canmore_cpp/LinuxClient.hpp"

#include "canmore/reg_mapped/interface/linux.h"

using namespace Canmore;

#define linux_itf_mode CANMORE_CONTROL_INTERFACE_MODE_LINUX

LinuxClient::LinuxClient(std::shared_ptr<RegMappedClient> client): client(client) {
    RegisterPage genCtrlPage(client, linux_itf_mode, CANMORE_LINUX_GEN_CONTROL_PAGE_NUM);

    // Make sure we're talking to what we expect to
    uint32_t debug_magic = genCtrlPage.readRegister(CANMORE_LINUX_GEN_CONTROL_MAGIC_OFFSET);
    if (debug_magic != CANMORE_LINUX_GEN_CONTROL_MAGIC_VALUE) {
        throw LinuxClientError("Unexpected value in general control magic for linux");
    }

    // Read flash ID to cache in class
    cachedFlashID.word[0] = genCtrlPage.readRegister(CANMORE_LINUX_GEN_CONTROL_LOWER_FLASH_ID);
    cachedFlashID.word[1] = genCtrlPage.readRegister(CANMORE_LINUX_GEN_CONTROL_UPPER_FLASH_ID);
}

std::string LinuxClient::getVersion() {
    return client->readStringPage(linux_itf_mode, CANMORE_LINUX_VERSION_STRING_PAGE_NUM);
}

void LinuxClient::enableRemoteTty(const std::string &term_env, uint16_t windowRows, uint16_t windowCols,
                                  const std::string &cmd) {
    // Write terminal environment variable
    if (term_env.size() >= REG_MAPPED_PAGE_SIZE) {
        throw CanmoreError("Terminal environment variable too large to transmit");
    }
    client->writeStringPage(linux_itf_mode, CANMORE_LINUX_TTY_TERMINAL_PAGE_NUM, term_env);
    client->writeStringPage(linux_itf_mode, CANMORE_LINUX_TTY_CMD_PAGE_NUM, cmd);

    // Set the initial window size, then enable the remote tty
    RegisterPage ttyCtrlPage(client, linux_itf_mode, CANMORE_LINUX_TTY_CONTROL_PAGE_NUM);
    uint32_t windowSize = ((uint32_t) windowRows) << 16 | windowCols;
    ttyCtrlPage.writeRegister(CANMORE_LINUX_TTY_CONTROL_WINDOW_SIZE_OFFSET, windowSize);
    ttyCtrlPage.writeRegister(CANMORE_LINUX_TTY_CONTROL_ENABLE_OFFSET, 1);
}

void LinuxClient::disableRemoteTty() {
    RegisterPage ttyCtrlPage(client, linux_itf_mode, CANMORE_LINUX_TTY_CONTROL_PAGE_NUM);
    ttyCtrlPage.writeRegister(CANMORE_LINUX_TTY_CONTROL_ENABLE_OFFSET, 0);
}

bool LinuxClient::remoteTtyEnabled() {
    RegisterPage ttyCtrlPage(client, linux_itf_mode, CANMORE_LINUX_TTY_CONTROL_PAGE_NUM);
    return ttyCtrlPage.readRegister(CANMORE_LINUX_TTY_CONTROL_ENABLE_OFFSET) != 0;
}

void LinuxClient::ping() {
    RegisterPage genCtrlPage(client, linux_itf_mode, CANMORE_LINUX_GEN_CONTROL_PAGE_NUM);

    // Verify it's still the correct magic value
    uint32_t debug_magic = genCtrlPage.readRegister(CANMORE_LINUX_GEN_CONTROL_MAGIC_OFFSET);
    if (debug_magic != CANMORE_LINUX_GEN_CONTROL_MAGIC_VALUE) {
        throw LinuxClientError("Unexpected value in general control magic for linux");
    }
}

void LinuxClient::reboot() {
    RegisterPage genCtrlPage(client, linux_itf_mode, CANMORE_LINUX_GEN_CONTROL_PAGE_NUM);
    genCtrlPage.writeRegister(CANMORE_LINUX_GEN_CONTROL_REBOOT_DEVICE_OFFSET,
                              CANMORE_LINUX_GEN_CONTROL_REBOOT_DEVICE_MAGIC);
}

void LinuxClient::restartDaemon() {
    RegisterPage genCtrlPage(client, linux_itf_mode, CANMORE_LINUX_GEN_CONTROL_PAGE_NUM);
    genCtrlPage.writeRegister(CANMORE_LINUX_GEN_CONTROL_RESTART_DAEMON_OFFSET,
                              CANMORE_LINUX_GEN_CONTROL_RESTART_DAEMON_MAGIC);
}
