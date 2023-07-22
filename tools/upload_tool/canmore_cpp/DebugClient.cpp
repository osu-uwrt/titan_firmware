#include "DebugClient.hpp"

#include "titan/canmore.h"

using namespace Canmore;

DebugClient::DebugClient(std::shared_ptr<RegMappedClient> client): client(client) {
    RegisterPage mcuCtrlPage(client, CANMORE_TITAN_CONTROL_INTERFACE_MODE_NORMAL, CANMORE_DBG_MCU_CONTROL_PAGE_NUM);

    // Make sure we're talking to what we expect to
    uint32_t debug_magic = mcuCtrlPage.readRegister(CANMORE_DBG_MCU_CONTROL_MAGIC_OFFSET);

    if (debug_magic != CANMORE_DBG_MCU_CONTROL_MAGIC_VALUE) {
        throw DebugError("Unexpected bootloader magic");
    }

    // Read flash ID to cache in class
    cachedFlashID.word[0] = mcuCtrlPage.readRegister(CANMORE_DBG_MCU_CONTROL_LOWER_FLASH_ID);
    cachedFlashID.word[1] = mcuCtrlPage.readRegister(CANMORE_DBG_MCU_CONTROL_UPPER_FLASH_ID);
}

void DebugClient::enterBootloader() {
    RegisterPage mcuCtrlPage(client, CANMORE_TITAN_CONTROL_INTERFACE_MODE_NORMAL, CANMORE_DBG_MCU_CONTROL_PAGE_NUM);
    mcuCtrlPage.writeRegister(CANMORE_DBG_MCU_CONTROL_ENTER_BL_OFFSET, 1);
}

std::string DebugClient::getVersion() {
    return client->readStringPage(CANMORE_TITAN_CONTROL_INTERFACE_MODE_NORMAL, CANMORE_DBG_VERSION_STRING_PAGE_NUM);
}
