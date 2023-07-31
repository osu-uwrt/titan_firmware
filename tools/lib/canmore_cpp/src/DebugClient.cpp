#include "titan/canmore.h"
#include "canmore_cpp/DebugClient.hpp"

using namespace Canmore;

DebugClient::DebugClient(std::shared_ptr<RegMappedClient> client): client(client) {
    RegisterPage mcuCtrlPage(client, CANMORE_TITAN_CONTROL_INTERFACE_MODE_NORMAL, CANMORE_DBG_MCU_CONTROL_PAGE_NUM);

    // Make sure we're talking to what we expect to
    uint32_t debug_magic = mcuCtrlPage.readRegister(CANMORE_DBG_MCU_CONTROL_MAGIC_OFFSET);

    if (debug_magic != CANMORE_DBG_MCU_CONTROL_MAGIC_VALUE) {
        throw DebugError("Unexpected application magic");
    }

    // Read flash ID to cache in class
    cachedFlashID.word[0] = mcuCtrlPage.readRegister(CANMORE_DBG_MCU_CONTROL_LOWER_FLASH_ID);
    cachedFlashID.word[1] = mcuCtrlPage.readRegister(CANMORE_DBG_MCU_CONTROL_UPPER_FLASH_ID);
}

void DebugClient::ping() {
    RegisterPage mcuCtrlPage(client, CANMORE_TITAN_CONTROL_INTERFACE_MODE_NORMAL, CANMORE_DBG_MCU_CONTROL_PAGE_NUM);
    uint32_t debug_magic = mcuCtrlPage.readRegister(CANMORE_DBG_MCU_CONTROL_MAGIC_OFFSET);

    if (debug_magic != CANMORE_DBG_MCU_CONTROL_MAGIC_VALUE) {
        throw DebugError("Unexpected application magic during ping");
    }
}

uint32_t DebugClient::getActiveFaults() {
    RegisterPage safetyStatusPage(client, CANMORE_TITAN_CONTROL_INTERFACE_MODE_NORMAL, CANMORE_DBG_SAFETY_STATUS_PAGE_NUM);
    return safetyStatusPage.readRegister(CANMORE_DBG_SAFETY_STATUS_FAULT_LIST_OFFSET);
}

SafetyStatus DebugClient::getSafetyStatus() {
    RegisterPage safetyStatusPage(client, CANMORE_TITAN_CONTROL_INTERFACE_MODE_NORMAL, CANMORE_DBG_SAFETY_STATUS_PAGE_NUM);
    return SafetyStatus(safetyStatusPage.readRegister(CANMORE_DBG_SAFETY_STATUS_GLOBAL_STATE_OFFSET));
}

std::string DebugClient::lookupFaultName(uint32_t faultId) {
    auto entry = faultNameCache.find(faultId);
    if (entry == faultNameCache.end()) {
        RegisterPage safetyStatusPage(client, CANMORE_TITAN_CONTROL_INTERFACE_MODE_NORMAL, CANMORE_DBG_SAFETY_STATUS_PAGE_NUM);
        safetyStatusPage.writeRegister(CANMORE_DBG_SAFETY_STATUS_FAULT_NAME_IDX_OFFSET, faultId);

        auto faultName = client->readStringPage(CANMORE_TITAN_CONTROL_INTERFACE_MODE_NORMAL, CANMORE_DBG_FAULT_NAME_PAGE_NUM);
        faultNameCache.emplace(faultId, faultName);
        return faultName;
    }
    else {
        return entry->second;
    }
}

void DebugClient::raiseFault(uint32_t faultId) {
    RegisterPage safetyStatusPage(client, CANMORE_TITAN_CONTROL_INTERFACE_MODE_NORMAL, CANMORE_DBG_SAFETY_STATUS_PAGE_NUM);
    safetyStatusPage.writeRegister(CANMORE_DBG_SAFETY_STATUS_RAISE_FAULT_OFFSET, faultId);
}

void DebugClient::lowerFault(uint32_t faultId) {
    RegisterPage safetyStatusPage(client, CANMORE_TITAN_CONTROL_INTERFACE_MODE_NORMAL, CANMORE_DBG_SAFETY_STATUS_PAGE_NUM);
    safetyStatusPage.writeRegister(CANMORE_DBG_SAFETY_STATUS_LOWER_FAULT_OFFSET, faultId);
}


Uptime DebugClient::getUptime() {
    RegisterPage safetyStatusPage(client, CANMORE_TITAN_CONTROL_INTERFACE_MODE_NORMAL, CANMORE_DBG_SAFETY_STATUS_PAGE_NUM);

    return Uptime(safetyStatusPage.readRegister(CANMORE_DBG_SAFETY_STATUS_UPTIME_OFFSET));
}

CrashCounter DebugClient::getCrashCounter() {
    RegisterPage crashLogPage(client, CANMORE_TITAN_CONTROL_INTERFACE_MODE_NORMAL, CANMORE_DBG_CRASH_LOG_PAGE_NUM);

    return CrashCounter(crashLogPage.readRegister(CANMORE_DBG_CRASH_LOG_CRASH_COUNT_OFFSET));
}

CrashLogEntry DebugClient::getLastResetEntry() {
    RegisterPage crashLogPage(client, CANMORE_TITAN_CONTROL_INTERFACE_MODE_NORMAL, CANMORE_DBG_CRASH_LOG_PAGE_NUM);

    // Select the most recent crash log entry
    crashLogPage.writeRegister(CANMORE_DBG_CRASH_LOG_PREV_INDEX_OFFSET, 0);

    // Read the crash log
    return CrashLogEntry(
        crashLogPage.readRegister(CANMORE_DBG_CRASH_LOG_RESET_REASON_OFFSET),
        crashLogPage.readRegister(CANMORE_DBG_CRASH_LOG_PREV_FAULT_LIST_OFFSET),
        crashLogPage.readRegister(CANMORE_DBG_CRASH_LOG_PREV_UPTIME_OFFSET),
        crashLogPage.readRegister(CANMORE_DBG_CRASH_LOG_PREV_SCRATCH_1_OFFSET),
        crashLogPage.readRegister(CANMORE_DBG_CRASH_LOG_PREV_SCRATCH_2_OFFSET));
}

void DebugClient::getCrashLog(std::vector<CrashLogEntry> &crashLogOut) {
    RegisterPage crashLogPage(client, CANMORE_TITAN_CONTROL_INTERFACE_MODE_NORMAL, CANMORE_DBG_CRASH_LOG_PAGE_NUM);

    uint32_t entryCount = crashLogPage.readRegister(CANMORE_DBG_CRASH_LOG_PREV_COUNT_OFFSET);

    crashLogOut.clear();
    for (uint32_t i = 0; i < entryCount; i++) {
        // Select the crash log entry
        crashLogPage.writeRegister(CANMORE_DBG_CRASH_LOG_PREV_INDEX_OFFSET, i);

        // Add this crash log entry
        crashLogOut.emplace_back(
            crashLogPage.readRegister(CANMORE_DBG_CRASH_LOG_RESET_REASON_OFFSET),
            crashLogPage.readRegister(CANMORE_DBG_CRASH_LOG_PREV_FAULT_LIST_OFFSET),
            crashLogPage.readRegister(CANMORE_DBG_CRASH_LOG_PREV_UPTIME_OFFSET),
            crashLogPage.readRegister(CANMORE_DBG_CRASH_LOG_PREV_SCRATCH_1_OFFSET),
            crashLogPage.readRegister(CANMORE_DBG_CRASH_LOG_PREV_SCRATCH_2_OFFSET));
    }
}

void DebugClient::reboot() {
    RegisterPage mcuCtrlPage(client, CANMORE_TITAN_CONTROL_INTERFACE_MODE_NORMAL, CANMORE_DBG_MCU_CONTROL_PAGE_NUM);
    mcuCtrlPage.writeRegister(CANMORE_DBG_MCU_CONTROL_REBOOT_MCU_OFFSET, 1);
}

void DebugClient::enterBootloader() {
    RegisterPage mcuCtrlPage(client, CANMORE_TITAN_CONTROL_INTERFACE_MODE_NORMAL, CANMORE_DBG_MCU_CONTROL_PAGE_NUM);
    mcuCtrlPage.writeRegister(CANMORE_DBG_MCU_CONTROL_ENTER_BL_OFFSET, 1);
}

std::string DebugClient::getVersion() {
    return client->readStringPage(CANMORE_TITAN_CONTROL_INTERFACE_MODE_NORMAL, CANMORE_DBG_VERSION_STRING_PAGE_NUM);
}
