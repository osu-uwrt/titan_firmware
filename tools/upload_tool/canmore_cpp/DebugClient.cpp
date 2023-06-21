#include "DebugClient.hpp"

#include "titan/canmore.h"

using namespace Canmore;

DebugClient::DebugClient(std::shared_ptr<RegMappedClient> client): client(client) {
    RegisterPage mcuCtrlPage(client, CANMORE_TITAN_CONTROL_INTERFACE_MODE_NORMAL, 0);

    // Make sure we're talking to what we expect to
    // TODO: Switch these to not hardcoded
    uint32_t debug_magic = mcuCtrlPage.readRegister(0);

    if (debug_magic != 0x0DBAA1F0) {
        throw DebugError("Unexpected bootloader magic");
    }

    // Read flash ID to cache in class
    cachedFlashID.word[0] = mcuCtrlPage.readRegister(2);
    cachedFlashID.word[1] = mcuCtrlPage.readRegister(3);
}

void DebugClient::enterBootloader() {
    RegisterPage mcuCtrlPage(client, CANMORE_TITAN_CONTROL_INTERFACE_MODE_NORMAL, 0);
    mcuCtrlPage.writeRegister(1, 1);
}