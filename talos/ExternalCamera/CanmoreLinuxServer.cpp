#include "DFCDaemon.hpp"

#define bind_reg_cb(func) std::bind(func, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3)

// TODO: Update to actual version
const char version_str[] = "<No Version Assigned>";

CanmoreLinuxServer::CanmoreLinuxServer(int ifIndex, uint8_t clientId):
    server(ifIndex, clientId, CANMORE_TITAN_CHAN_CONTROL_INTERFACE, CANMORE_TITAN_CONTROL_INTERFACE_MODE_LINUX) {
    // Define general control page
    auto gen_control = Canmore::RegMappedRegisterPage::create();
    gen_control->addConstRegister(CANMORE_LINUX_GEN_CONTROL_MAGIC_OFFSET, CANMORE_LINUX_GEN_CONTROL_MAGIC_VALUE);
    gen_control->addConstRegister(CANMORE_LINUX_GEN_CONTROL_LOWER_FLASH_ID, 0xEFBEADDE);
    gen_control->addConstRegister(CANMORE_LINUX_GEN_CONTROL_UPPER_FLASH_ID, 0xCEFAEDFE);

    gen_control->addCallbackRegister(CANMORE_LINUX_GEN_CONTROL_RESTART_DAEMON_OFFSET, REGISTER_PERM_WRITE_ONLY,
                                     bind_reg_cb(&CanmoreLinuxServer::restartDaemonCb));

    server.addRegisterPage(CANMORE_LINUX_GEN_CONTROL_PAGE_NUM, std::move(gen_control));

    server.addByteMappedPage(CANMORE_LINUX_VERSION_STRING_PAGE_NUM, REGISTER_PERM_READ_ONLY,
                             std::span<uint8_t>((uint8_t *) version_str, sizeof(version_str)));
}

void CanmoreLinuxServer::run() {
    while (!shouldStop) {
        server.waitForPacket(1000);
    }
}

bool CanmoreLinuxServer::restartDaemonCb(uint16_t addr, bool is_write, uint32_t *data_ptr) {
    if (*data_ptr == CANMORE_LINUX_GEN_CONTROL_RESTART_DAEMON_MAGIC) {
        shouldStop = true;
        return true;
    }
    else {
        return false;
    }
}
