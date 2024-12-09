#include "canmore_cpp/DebugClient.hpp"

#include "canmore/reg_mapped/interface/debug.h"

using namespace Canmore;

#define debug_itf_mode CANMORE_CONTROL_INTERFACE_MODE_NORMAL

DebugClient::DebugClient(std::shared_ptr<RegMappedClient> client): client(client) {
    RegisterPage mcuCtrlPage(client, debug_itf_mode, CANMORE_DBG_MCU_CONTROL_PAGE_NUM);

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
    RegisterPage mcuCtrlPage(client, debug_itf_mode, CANMORE_DBG_MCU_CONTROL_PAGE_NUM);
    uint32_t debug_magic = mcuCtrlPage.readRegister(CANMORE_DBG_MCU_CONTROL_MAGIC_OFFSET);

    if (debug_magic != CANMORE_DBG_MCU_CONTROL_MAGIC_VALUE) {
        throw DebugError("Unexpected application magic during ping");
    }
}

uint32_t DebugClient::getActiveFaults() {
    RegisterPage safetyStatusPage(client, debug_itf_mode, CANMORE_DBG_SAFETY_STATUS_PAGE_NUM);
    return safetyStatusPage.readRegister(CANMORE_DBG_SAFETY_STATUS_FAULT_LIST_OFFSET);
}

SafetyStatus DebugClient::getSafetyStatus() {
    RegisterPage safetyStatusPage(client, debug_itf_mode, CANMORE_DBG_SAFETY_STATUS_PAGE_NUM);
    return SafetyStatus(safetyStatusPage.readRegister(CANMORE_DBG_SAFETY_STATUS_GLOBAL_STATE_OFFSET));
}

std::string DebugClient::lookupFaultName(uint32_t faultId) {
    auto entry = faultNameCache.find(faultId);
    if (entry == faultNameCache.end()) {
        RegisterPage safetyStatusPage(client, debug_itf_mode, CANMORE_DBG_SAFETY_STATUS_PAGE_NUM);
        safetyStatusPage.writeRegister(CANMORE_DBG_SAFETY_STATUS_FAULT_IDX_OFFSET, faultId);

        auto faultName = client->readStringPage(debug_itf_mode, CANMORE_DBG_FAULT_NAME_PAGE_NUM);
        faultNameCache.emplace(faultId, faultName);
        return faultName;
    }
    else {
        return entry->second;
    }
}

FaultData DebugClient::lookupFaultData(uint32_t faultId) {
    RegisterPage safetyStatusPage(client, debug_itf_mode, CANMORE_DBG_SAFETY_STATUS_PAGE_NUM);
    RegisterPage faultDataPage(client, debug_itf_mode, CANMORE_DBG_FAULT_DATA_PAGE_NUM);

    // Select fault to read
    safetyStatusPage.writeRegister(CANMORE_DBG_SAFETY_STATUS_FAULT_IDX_OFFSET, faultId);

    // Read the data
    bool sticky = faultDataPage.readRegister(CANMORE_DBG_FAULT_DATA_STICKY_OFFSET) ? true : false;
    bool multipleFires = faultDataPage.readRegister(CANMORE_DBG_FAULT_DATA_MULTIPLE_FIRES_OFFSET) ? true : false;
    uint32_t timestampLower = faultDataPage.readRegister(CANMORE_DBG_FAULT_DATA_TIME_LOWER_OFFSET);
    uint32_t timestampUpper = faultDataPage.readRegister(CANMORE_DBG_FAULT_DATA_TIME_UPPER_OFFSET);
    uint32_t extraData = faultDataPage.readRegister(CANMORE_DBG_FAULT_DATA_EXTRA_DATA_OFFSET);
    uint16_t line = faultDataPage.readRegister(CANMORE_DBG_FAULT_DATA_LINE_NUMBER_OFFSET);
    std::string filename = client->readStringPage(debug_itf_mode, CANMORE_DBG_FAULT_FILENAME_PAGE_NUM);

    return FaultData(faultId, lookupFaultName(faultId), sticky, multipleFires, timestampLower, timestampUpper,
                     extraData, filename, line);
}

void DebugClient::raiseFault(uint32_t faultId) {
    RegisterPage safetyStatusPage(client, debug_itf_mode, CANMORE_DBG_SAFETY_STATUS_PAGE_NUM);
    safetyStatusPage.writeRegister(CANMORE_DBG_SAFETY_STATUS_RAISE_FAULT_OFFSET, faultId);
}

void DebugClient::lowerFault(uint32_t faultId) {
    RegisterPage safetyStatusPage(client, debug_itf_mode, CANMORE_DBG_SAFETY_STATUS_PAGE_NUM);
    safetyStatusPage.writeRegister(CANMORE_DBG_SAFETY_STATUS_LOWER_FAULT_OFFSET, faultId);
}

Uptime DebugClient::getUptime() {
    RegisterPage safetyStatusPage(client, debug_itf_mode, CANMORE_DBG_SAFETY_STATUS_PAGE_NUM);

    return Uptime(safetyStatusPage.readRegister(CANMORE_DBG_SAFETY_STATUS_UPTIME_OFFSET));
}

CrashCounter DebugClient::getCrashCounter() {
    RegisterPage crashLogPage(client, debug_itf_mode, CANMORE_DBG_CRASH_LOG_PAGE_NUM);

    return CrashCounter(crashLogPage.readRegister(CANMORE_DBG_CRASH_LOG_CRASH_COUNT_OFFSET));
}

CrashLogEntry DebugClient::getLastResetEntry() {
    RegisterPage crashLogPage(client, debug_itf_mode, CANMORE_DBG_CRASH_LOG_PAGE_NUM);

    // Select the most recent crash log entry
    crashLogPage.writeRegister(CANMORE_DBG_CRASH_LOG_PREV_INDEX_OFFSET, 0);

    // Read the crash log
    return CrashLogEntry(crashLogPage.readRegister(CANMORE_DBG_CRASH_LOG_RESET_REASON_OFFSET),
                         crashLogPage.readRegister(CANMORE_DBG_CRASH_LOG_PREV_FAULT_LIST_OFFSET),
                         crashLogPage.readRegister(CANMORE_DBG_CRASH_LOG_PREV_UPTIME_OFFSET),
                         crashLogPage.readRegister(CANMORE_DBG_CRASH_LOG_PREV_SCRATCH_1_OFFSET),
                         crashLogPage.readRegister(CANMORE_DBG_CRASH_LOG_PREV_SCRATCH_2_OFFSET));
}

void DebugClient::getCrashLog(std::vector<CrashLogEntry> &crashLogOut) {
    RegisterPage crashLogPage(client, debug_itf_mode, CANMORE_DBG_CRASH_LOG_PAGE_NUM);

    uint32_t entryCount = crashLogPage.readRegister(CANMORE_DBG_CRASH_LOG_PREV_COUNT_OFFSET);

    crashLogOut.clear();
    for (uint32_t i = 0; i < entryCount; i++) {
        // Select the crash log entry
        crashLogPage.writeRegister(CANMORE_DBG_CRASH_LOG_PREV_INDEX_OFFSET, i);

        // Add this crash log entry
        crashLogOut.emplace_back(crashLogPage.readRegister(CANMORE_DBG_CRASH_LOG_RESET_REASON_OFFSET),
                                 crashLogPage.readRegister(CANMORE_DBG_CRASH_LOG_PREV_FAULT_LIST_OFFSET),
                                 crashLogPage.readRegister(CANMORE_DBG_CRASH_LOG_PREV_UPTIME_OFFSET),
                                 crashLogPage.readRegister(CANMORE_DBG_CRASH_LOG_PREV_SCRATCH_1_OFFSET),
                                 crashLogPage.readRegister(CANMORE_DBG_CRASH_LOG_PREV_SCRATCH_2_OFFSET));
    }
}

uint32_t DebugClient::readMemory(uint32_t addr) {
    RegisterPage gdbStubPage(client, debug_itf_mode, CANMORE_DBG_GDB_STUB_PAGE_NUM);
    gdbStubPage.writeRegister(CANMORE_DBG_GDB_STUB_READ_WORD_ADDR_OFFSET, addr);
    return gdbStubPage.readRegister(CANMORE_DBG_GDB_STUB_MEMORY_DATA_OFFSET);
}

void DebugClient::writeMemory(uint32_t addr, uint32_t data) {
    RegisterPage gdbStubPage(client, debug_itf_mode, CANMORE_DBG_GDB_STUB_PAGE_NUM);
    gdbStubPage.writeRegister(CANMORE_DBG_GDB_STUB_MEMORY_DATA_OFFSET, data);
    gdbStubPage.writeRegister(CANMORE_DBG_GDB_STUB_WRITE_WORD_ADDR_OFFSET, addr);
}

uint32_t DebugClient::getGDBStubPC() {
    RegisterPage gdbStubPage(client, debug_itf_mode, CANMORE_DBG_GDB_STUB_PAGE_NUM);
    return gdbStubPage.readRegister(CANMORE_DBG_GDB_STUB_PC_REGISTER_OFFSET);
}

uint32_t DebugClient::getGDBStubSP() {
    RegisterPage gdbStubPage(client, debug_itf_mode, CANMORE_DBG_GDB_STUB_PAGE_NUM);
    return gdbStubPage.readRegister(CANMORE_DBG_GDB_STUB_SP_REGISTER_OFFSET);
}

uint32_t DebugClient::getGDBStubLR() {
    RegisterPage gdbStubPage(client, debug_itf_mode, CANMORE_DBG_GDB_STUB_PAGE_NUM);
    return gdbStubPage.readRegister(CANMORE_DBG_GDB_STUB_LR_REGISTER_OFFSET);
}

void DebugClient::reboot() {
    RegisterPage mcuCtrlPage(client, debug_itf_mode, CANMORE_DBG_MCU_CONTROL_PAGE_NUM);
    mcuCtrlPage.writeRegister(CANMORE_DBG_MCU_CONTROL_REBOOT_MCU_OFFSET, 1);
}

void DebugClient::enterBootloader() {
    RegisterPage mcuCtrlPage(client, debug_itf_mode, CANMORE_DBG_MCU_CONTROL_PAGE_NUM);
    mcuCtrlPage.writeRegister(CANMORE_DBG_MCU_CONTROL_ENTER_BL_OFFSET, 1);
}

std::string DebugClient::getVersion() {
    return client->readStringPage(debug_itf_mode, CANMORE_DBG_VERSION_STRING_PAGE_NUM);
}

int DebugClient::executeRemoteCmd(std::vector<std::string> const &args, std::string &response) {
    // Create the compressed arg string to send to the client
    std::stringstream argStream;
    for (std::string const &arg : args) {
        argStream << arg << '\0';
    }
    argStream << '\0';

    // Verify that the args can fit into the register page
    const std::string &argCompressed = argStream.str();
    if (argCompressed.length() > CANMORE_DBG_REMOTE_CMD_ARGS_MAX_LEN) {
        throw DebugError("Remote command arguments are too large for remote command register on device");
    }

    // Write the string into the page
    client->writeStringPage(debug_itf_mode, CANMORE_DBG_REMOTE_CMD_ARGS_PAGE_NUM, argCompressed);

    // Execute the command
    RegisterPage remoteCmdPage(client, debug_itf_mode, CANMORE_DBG_REMOTE_CMD_PAGE_NUM);
    int32_t rc = (int32_t) remoteCmdPage.readRegister(CANMORE_DBG_REMOTE_CMD_EXECUTE_OFFSET);

    // Read back the resulting string
    response = client->readStringPage(debug_itf_mode, CANMORE_DBG_REMOTE_CMD_RESP_PAGE_NUM);
    return rc;
}
