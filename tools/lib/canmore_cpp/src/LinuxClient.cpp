#include "canmore_cpp/LinuxClient.hpp"

using namespace Canmore;

#define linux_itf_mode CANMORE_TITAN_CONTROL_INTERFACE_MODE_LINUX

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
