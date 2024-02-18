#pragma once

#include "Canmore.hpp"
#include "GDBClient.hpp"
#include "RegMappedClient.hpp"

#include <array>
#include <bitset>
#include <iomanip>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

namespace Canmore {

class DebugError : public CanmoreError {
    using CanmoreError::CanmoreError;
};

struct SafetyStatus {
    const bool safetySetup;
    const bool safetyInitialized;
    const bool killIsEnabled;
    const bool faultPresent;

    SafetyStatus(uint32_t safetyStatusField):
        safetySetup((safetyStatusField & (1 << CANMORE_DBG_SAFETY_STATUS_GLOBAL_STATE_IS_SETUP_FIELD)) != 0),
        safetyInitialized((safetyStatusField & (1 << CANMORE_DBG_SAFETY_STATUS_GLOBAL_STATE_IS_INITIALIZED_FIELD)) !=
                          0),
        killIsEnabled((safetyStatusField & (1 << CANMORE_DBG_SAFETY_STATUS_GLOBAL_STATE_KILL_IS_ENABLED_FIELD)) != 0),
        faultPresent((safetyStatusField & (1 << CANMORE_DBG_SAFETY_STATUS_GLOBAL_STATE_FAULT_PRESENT_FIELD)) != 0) {}

    std::string getGlobalState() const {
        if (killIsEnabled) {
            if (safetySetup && safetyInitialized) {
                return "Robot Enabled";
            }
            else {
                return "Invalid? Kill Enabled";
            }
        }
        else if (safetyInitialized) {
            if (safetySetup) {
                return "ROS Connected";
            }
            else {
                return "Invalid? Safety initialized";
            }
        }
        else {
            return "Disconnected - Idle";
        }
    }
};

struct Uptime {
    const uint32_t uptimeRaw;

    Uptime(uint32_t uptimeField): uptimeRaw(uptimeField) {}

    std::string format() const {
        std::stringstream formatStream;
        bool hadPrint = false;
        uint32_t seconds = uptimeRaw / CANMORE_DBG_SAFETY_STATUS_UPTIME_TICKS_PER_SECOND;
        uint32_t days = seconds / (3600 * 24);
        seconds %= 3600 * 24;
        uint32_t hours = seconds / 3600;
        seconds %= 3600;
        uint32_t minutes = seconds / 60;
        seconds %= 60;

        if (days > 0 || hadPrint) {
            formatStream << days << " days ";
            hadPrint = true;
        }
        if (hours > 0 || hadPrint) {
            formatStream << hours << " hours ";
            hadPrint = true;
        }
        if (minutes > 0 || hadPrint) {
            formatStream << minutes << " minutes ";
            hadPrint = true;
        }
        formatStream << seconds << " seconds";
        return formatStream.str();
    }

    std::string formatSeconds() const {
        std::stringstream formatStream;
        uint32_t seconds = uptimeRaw / CANMORE_DBG_SAFETY_STATUS_UPTIME_TICKS_PER_SECOND;
        uint32_t fraction = uptimeRaw % CANMORE_DBG_SAFETY_STATUS_UPTIME_TICKS_PER_SECOND;
        static_assert(CANMORE_DBG_SAFETY_STATUS_UPTIME_TICKS_PER_SECOND == 100, "Decimal width doesn't match");
        formatStream << seconds << "." << std::setw(2) << std::setfill('0') << fraction;
        return formatStream.str();
    }
};

struct CrashCounter {
    const uint8_t totalCount;
    const uint8_t panicCount;
    const uint8_t hardFaultCount;
    const uint8_t assertFailCount;

    CrashCounter(uint32_t crashCountField):
        totalCount((crashCountField >> CANMORE_DBG_CRASH_LOG_CRASH_COUNT_TOTAL_SHIFT) &
                   CANMORE_DBG_CRASH_LOG_CRASH_COUNT_TOTAL_MASK),
        panicCount((crashCountField >> CANMORE_DBG_CRASH_LOG_CRASH_COUNT_PANIC_SHIFT) &
                   CANMORE_DBG_CRASH_LOG_CRASH_COUNT_PANIC_MASK),
        hardFaultCount((crashCountField >> CANMORE_DBG_CRASH_LOG_CRASH_COUNT_HARD_FAULT_SHIFT) &
                       CANMORE_DBG_CRASH_LOG_CRASH_COUNT_HARD_FAULT_MASK),
        assertFailCount((crashCountField >> CANMORE_DBG_CRASH_LOG_CRASH_COUNT_ASSERT_FAIL_SHIFT) &
                        CANMORE_DBG_CRASH_LOG_CRASH_COUNT_ASSERT_FAIL_MASK) {}

    std::string format() const {
        std::stringstream countStream;
        countStream << "Total Crashes: " << (int) totalCount;
        countStream << " - Panics: " << (int) panicCount;
        countStream << " - Hard Faults: " << (int) hardFaultCount;
        countStream << " - Assert Fails: " << (int) assertFailCount;
        return countStream.str();
    }
};

struct CrashLogEntry {
    const uint32_t resetReason;
    const uint32_t faults;
    const uint32_t uptime;
    const uint32_t scratch1;
    const uint32_t scratch2;

    CrashLogEntry(uint32_t resetReason, uint32_t faults, uint32_t uptime, uint32_t scratch1, uint32_t scratch2):
        resetReason(resetReason), faults(faults), uptime(uptime), scratch1(scratch1), scratch2(scratch2) {}

    std::string format() const {
        std::stringstream reasonStream;
        reasonStream << std::hex << std::setw(8) << std::setfill('0');
        if (resetReason == CANMORE_DBG_CRASH_LOG_RESET_REASON_CLEAN_BOOT_VALUE) {
            reasonStream << "Clean Boot: ";

            if (scratch1 == CANMORE_DBG_CRASH_LOG_PREV_SCRATCH_1_CLEAN_RESET_POR_VALUE) {
                reasonStream << "Power-on Reset";
            }
            else if (scratch1 == CANMORE_DBG_CRASH_LOG_PREV_SCRATCH_1_CLEAN_RESET_RUN_VALUE) {
                reasonStream << "RESET Pin";
            }
            else if (scratch1 == CANMORE_DBG_CRASH_LOG_PREV_SCRATCH_1_CLEAN_RESET_PSM_VALUE) {
                reasonStream << "PSM Debug Request";
            }
            else if (scratch1 == CANMORE_DBG_CRASH_LOG_PREV_SCRATCH_1_CLEAN_RESET_SOFTWARE_VALUE) {
                reasonStream << "Software Request";
            }
            else if (scratch1 == CANMORE_DBG_CRASH_LOG_PREV_SCRATCH_1_CLEAN_RESET_UNK_WDG_VALUE) {
                reasonStream << "Unknown Watchdog Request";
            }
            else if (scratch1 == CANMORE_DBG_CRASH_LOG_PREV_SCRATCH_1_CLEAN_RESET_BOOTLOADER_VALUE) {
                reasonStream << "Bootloader";
            }
            else {
                reasonStream << "Unknown Clean Boot Type (0x" << scratch1 << ")";
            }
        }
        else {
            if (resetReason == CANMORE_DBG_CRASH_LOG_RESET_REASON_UNKNOWN_PREINIT_VALUE) {
                reasonStream << "Unexpected Reset before Init";
            }
            else if (resetReason == CANMORE_DBG_CRASH_LOG_RESET_REASON_UNKNOWN_ACTIVE_VALUE) {
                reasonStream << "Unexpected Reset while Active";
            }
            else if (resetReason == CANMORE_DBG_CRASH_LOG_RESET_REASON_PANIC_VALUE) {
                reasonStream << "Panic (Message: 0x" << scratch1 << " Call Address: 0x" << scratch2 << ")";
            }
            else if (resetReason == CANMORE_DBG_CRASH_LOG_RESET_REASON_HARD_FAULT_VALUE) {
                reasonStream << "Hard Fault (Fault Address: 0x" << scratch1 << ")";
            }
            else if (resetReason == CANMORE_DBG_CRASH_LOG_RESET_REASON_ASSERT_FAIL_VALUE) {
                reasonStream << "Assertion Fail (File: 0x" << scratch1 << " Line: 0x" << scratch2 << ")";
            }
            else if (resetReason == CANMORE_DBG_CRASH_LOG_RESET_REASON_HARD_ASSERT_VALUE) {
                reasonStream << "Hard Assert Fail (Fault Address: 0x" << scratch1 << ")";
            }
            else if (resetReason == CANMORE_DBG_CRASH_LOG_RESET_REASON_WATCHDOG_TIMEOUT_VALUE) {
                reasonStream << "Watchdog Timeout (Last Address: 0x" << scratch1 << ")";
            }
            else if (resetReason == CANMORE_DBG_CRASH_LOG_RESET_REASON_CORE1_TIMEOUT_VALUE) {
                reasonStream << "Core 1 Timeout (Last Address: 0x" << scratch1 << ")";
            }
            else {
                reasonStream << "Unknown Reset Reason (0x" << resetReason << ")";
            }

            reasonStream << " - Faults: 0x" << faults;
        }

        if (uptime) {
            const int uptime_ticks_per_sec = CANMORE_DBG_SAFETY_STATUS_UPTIME_TICKS_PER_SECOND;
            float uptime_pretty;
            char uptime_units;
            if (uptime > 3600 * 100) {
                uptime_pretty = (float) (uptime) / (3600 * uptime_ticks_per_sec);
                uptime_units = 'h';
            }
            else if (uptime > 60 * uptime_ticks_per_sec) {
                uptime_pretty = (float) (uptime) / (60 * uptime_ticks_per_sec);
                uptime_units = 'm';
            }
            else {
                uptime_pretty = (float) (uptime) / uptime_ticks_per_sec;
                uptime_units = 's';
            }
            reasonStream << " - Prev. Uptime: " << std::setw(0) << std::setprecision(2) << uptime_pretty
                         << uptime_units;
        }

        return reasonStream.str();
    }
};

struct FaultData {
    uint32_t faultId;
    std::string name;
    bool sticky;
    bool multipleFires;
    uint64_t timestamp;
    uint32_t extraData;
    std::string filename;
    uint16_t line;

    FaultData(uint32_t faultId, const std::string &name, bool sticky, bool multipleFires, uint32_t timestampLower,
              uint32_t timestampUpper, uint32_t extraData, const std::string &filename, uint16_t line):
        faultId(faultId),
        name(name), sticky(sticky), multipleFires(multipleFires),
        timestamp((((uint64_t) timestampUpper) << 32) + timestampLower), extraData(extraData), filename(filename),
        line(line) {}

    std::string formatTimestamp() const {
        std::stringstream out;
        out << std::setfill('0');

        // Timestamp in microseconds
        uint64_t timestamp_seconds = timestamp / 1000000;
        uint64_t timestamp_us = timestamp % 1000000;

        // Only display hours if we've gone that far
        if (timestamp_seconds > 3600) {
            out << timestamp_seconds / 3600 << ":" << std::setw(2);
            timestamp_seconds %= 3600;
        }

        uint64_t timestamp_minutes = timestamp_seconds / 60;
        timestamp_seconds %= 60;
        if (timestamp_minutes > 0) {
            out << timestamp_minutes << ":" << std::setw(2);
        }
        out << timestamp_seconds;

        // Add the microseconds
        out << "." << std::setw(6) << timestamp_us;
        return out.str();
    }
};

class DebugClient : public GDBClient {
public:
    DebugClient(std::shared_ptr<RegMappedClient> client);

    std::string getVersion();
    uint64_t getFlashId() { return cachedFlashID.doubleword; }
    void enterBootloader();
    void reboot();
    void ping() override;

    // Remote Command Execution
    int executeRemoteCmd(std::vector<std::string> const &args, std::string &response);

    // Misc Debug Information
    SafetyStatus getSafetyStatus();
    Uptime getUptime();
    CrashCounter getCrashCounter();
    CrashLogEntry getLastResetEntry();
    void getCrashLog(std::vector<CrashLogEntry> &crashLogOut);

    // GDB Bindings
    uint32_t readMemory(uint32_t addr) override;
    void writeMemory(uint32_t addr, uint32_t data) override;
    uint32_t getGDBStubPC() override;
    uint32_t getGDBStubSP() override;
    uint32_t getGDBStubLR() override;

    // Safety Fault Bindings
    std::string lookupFaultName(uint32_t faultId);
    FaultData lookupFaultData(uint32_t faultId);
    uint32_t getActiveFaults();
    void raiseFault(uint32_t faultId);
    void lowerFault(uint32_t faultId);

private:
    union flash_id cachedFlashID;
    std::shared_ptr<RegMappedClient> client;
    std::map<uint32_t, std::string> faultNameCache;
};

};  // namespace Canmore
