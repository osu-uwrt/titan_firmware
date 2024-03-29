#include "safety_internal.h"

#include "hardware/exception.h"
#include "hardware/structs/psm.h"
#include "hardware/structs/systick.h"
#include "hardware/structs/vreg_and_chip_reset.h"
#include "hardware/structs/watchdog.h"
#include "hardware/sync.h"
#include "hardware/watchdog.h"

#include <assert.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#if LIB_PICO_PRINTF_PICO
#include "pico/printf.h"
#else
#define weak_raw_printf printf
#define weak_raw_vprintf vprintf
#endif

// Use of Watchdog Scratch Registers
// Can really only be considered valid if watchdog_enable_caused_reboot is true
// scratch[0]: Last Crash Action
//  - CLEAN_BOOT: Last reset was a clean boot. Note that if this set, then a software requested reset occurred.
//                This is primarily used in the log to denote when the first clean boot occurred
//     scratch[1]: The reset cause (note the power on reset/run pin isn't actually in this register, rather only used in
//     the crash log)
//       - 0x7193001: Power on reset
//       - 0x7193002: Reset from RUN pin
//       - 0x7193003: Reset from debug emergency reset
//       - 0x7193004: Reset from software requested reset
//       - 0x7193005: Reset from unknown (non-timeout) watchdog reset: watchdog reporting a reset, but pico sdk didn't
//                    see the WATCHDOG_NON_REBOOT_MAGIC in the scratch registers
//       - 0x7193006: Reset from bootloader mode
//       - 0x7193007: Reset from RP2040 Bootrom (set after uploading w/ upload tool)
//  - UNKNOWN_SAFETY_PREINIT: Unknown, crashed after safety_setup
//  - UNKNOWN_SAFETY_ACTIVE: Unknown, crashed after safety_init
//  - PANIC: Set on panic function call
//     scratch[1]: Address of panic string (Won't be dereferenced for safety)
//  - HARD_FAULT: Set in the hard fault exception handler (Any unhandled exception raises this)
//     scratch[1]: Faulting Address
//  - ASSERT_FAIL: Set in assertion callback
//     scratch[1]: Faulting File String Address
//     scratch[2]: Faulting File Line
//  - HARD_ASSERT: Hard assertion failure when debugging not enabled
//     scratch[1]: Caller's address
//  - WATCHDOG_TIMEOUT: Watchdog timed out (However we caught it with the NMI 1us before the watchdog fired)
//     scratch[1]; The interrupted instruction right before the watchdog reset
//  - CORE1_TIMEOUT: Core 1 did not check in within SAFETY_CORE1_CHECKIN_INTERVAL_MS, and safety panicked
//     scratch[1]: The address of the interrupted instruction on core 1
// scratch[3]: Uptime since safety init (tens of milliseconds)
//             Note that with 10ms per pulse, this will overflow if running for ~1.3 years
// scratch[4]: Reserved for Pico SDK watchdog use
// !!! Note !!! Be careful about scratch[4] through scratch[7]. These are used by the watchdog reboot to set target addr
//     If you write to these variables, ensure that they DO NOT MODIFY these registers after watchdog_reboot is called
//     Failure to do so will result in erratic behavior while trying to reboot MCU into bootloader/bootrom modes

// Note the values are defined in safety_magic_values.h
#include "titan/safety_magic_values.h"

// ========================================
// Crash Log Definitions
// ========================================

struct crash_data crash_data __attribute__((section(".uninitialized_data.crash_data")));

// Ensure struct packed properly
static_assert(offsetof(struct crash_data, header.crc) == 0, "CRC must be first entry in crash data");
static_assert(sizeof(crash_data.header) == sizeof(crash_data.header.i), "Failed to pack header");
static_assert(sizeof(crash_data.crash_counter) == sizeof(crash_data.crash_counter.i), "Failed to pack crash counter");

static uint16_t crc16_poly_table[] = {
    0x0000, 0x1021, 0x2042, 0x3063, 0x4084, 0x50A5, 0x60C6, 0x70E7, 0x8108, 0x9129, 0xA14A, 0xB16B, 0xC18C, 0xD1AD,
    0xE1CE, 0xF1EF, 0x1231, 0x0210, 0x3273, 0x2252, 0x52B5, 0x4294, 0x72F7, 0x62D6, 0x9339, 0x8318, 0xB37B, 0xA35A,
    0xD3BD, 0xC39C, 0xF3FF, 0xE3DE, 0x2462, 0x3443, 0x0420, 0x1401, 0x64E6, 0x74C7, 0x44A4, 0x5485, 0xA56A, 0xB54B,
    0x8528, 0x9509, 0xE5EE, 0xF5CF, 0xC5AC, 0xD58D, 0x3653, 0x2672, 0x1611, 0x0630, 0x76D7, 0x66F6, 0x5695, 0x46B4,
    0xB75B, 0xA77A, 0x9719, 0x8738, 0xF7DF, 0xE7FE, 0xD79D, 0xC7BC, 0x48C4, 0x58E5, 0x6886, 0x78A7, 0x0840, 0x1861,
    0x2802, 0x3823, 0xC9CC, 0xD9ED, 0xE98E, 0xF9AF, 0x8948, 0x9969, 0xA90A, 0xB92B, 0x5AF5, 0x4AD4, 0x7AB7, 0x6A96,
    0x1A71, 0x0A50, 0x3A33, 0x2A12, 0xDBFD, 0xCBDC, 0xFBBF, 0xEB9E, 0x9B79, 0x8B58, 0xBB3B, 0xAB1A, 0x6CA6, 0x7C87,
    0x4CE4, 0x5CC5, 0x2C22, 0x3C03, 0x0C60, 0x1C41, 0xEDAE, 0xFD8F, 0xCDEC, 0xDDCD, 0xAD2A, 0xBD0B, 0x8D68, 0x9D49,
    0x7E97, 0x6EB6, 0x5ED5, 0x4EF4, 0x3E13, 0x2E32, 0x1E51, 0x0E70, 0xFF9F, 0xEFBE, 0xDFDD, 0xCFFC, 0xBF1B, 0xAF3A,
    0x9F59, 0x8F78, 0x9188, 0x81A9, 0xB1CA, 0xA1EB, 0xD10C, 0xC12D, 0xF14E, 0xE16F, 0x1080, 0x00A1, 0x30C2, 0x20E3,
    0x5004, 0x4025, 0x7046, 0x6067, 0x83B9, 0x9398, 0xA3FB, 0xB3DA, 0xC33D, 0xD31C, 0xE37F, 0xF35E, 0x02B1, 0x1290,
    0x22F3, 0x32D2, 0x4235, 0x5214, 0x6277, 0x7256, 0xB5EA, 0xA5CB, 0x95A8, 0x8589, 0xF56E, 0xE54F, 0xD52C, 0xC50D,
    0x34E2, 0x24C3, 0x14A0, 0x0481, 0x7466, 0x6447, 0x5424, 0x4405, 0xA7DB, 0xB7FA, 0x8799, 0x97B8, 0xE75F, 0xF77E,
    0xC71D, 0xD73C, 0x26D3, 0x36F2, 0x0691, 0x16B0, 0x6657, 0x7676, 0x4615, 0x5634, 0xD94C, 0xC96D, 0xF90E, 0xE92F,
    0x99C8, 0x89E9, 0xB98A, 0xA9AB, 0x5844, 0x4865, 0x7806, 0x6827, 0x18C0, 0x08E1, 0x3882, 0x28A3, 0xCB7D, 0xDB5C,
    0xEB3F, 0xFB1E, 0x8BF9, 0x9BD8, 0xABBB, 0xBB9A, 0x4A75, 0x5A54, 0x6A37, 0x7A16, 0x0AF1, 0x1AD0, 0x2AB3, 0x3A92,
    0xFD2E, 0xED0F, 0xDD6C, 0xCD4D, 0xBDAA, 0xAD8B, 0x9DE8, 0x8DC9, 0x7C26, 0x6C07, 0x5C64, 0x4C45, 0x3CA2, 0x2C83,
    0x1CE0, 0x0CC1, 0xEF1F, 0xFF3E, 0xCF5D, 0xDF7C, 0xAF9B, 0xBFBA, 0x8FD9, 0x9FF8, 0x6E17, 0x7E36, 0x4E55, 0x5E74,
    0x2E93, 0x3EB2, 0x0ED1, 0x1EF0,
};

static uint16_t calc_crc16(void *data, size_t size) {
    uint16_t crc16 = 0x1541;

    uint8_t *data_bytes = data;
    while (size > 0) {
        uint8_t lookup_index = ((crc16 >> 8) ^ *(data_bytes++)) & 0xFF;
        crc16 = (crc16 << 8) ^ crc16_poly_table[lookup_index];
        size--;
    }

    return crc16;
}

static uint16_t calc_crash_data_crc(void) {
    const size_t crc_size = sizeof(crash_data.header.crc);
    uintptr_t crash_data_check_start = ((uintptr_t) &crash_data) + crc_size;
    return calc_crc16((void *) crash_data_check_start, sizeof(crash_data) - crc_size);
}

// ========================================
// Watchdog Current-Session Crash Reporting
// ========================================

#define uptime_ticks_per_sec 100
static volatile uint32_t *reset_reason_reg = &watchdog_hw->scratch[0];

static_assert(PICO_SPINLOCK_ID_OS1 == SAFETY_CRASH_LOGGED_SPINLOCK_ID, "Fault spinlock does not match expected lock #");

// Put fault_list in memory, it should be fine to just live in a special location in memory, as this is only read across
// reboots used by the *same* firmware version (we won't have multiple firmware versions care about fault_list)
// Trying to keep out of the watchdog registers 4-7 (since special care is needed which can't be gaurenteed with
// fault_list)
volatile uint32_t fault_list __attribute__((section(".uninitialized_data.fault_list")));
volatile uint32_t *const fault_list_reg = &fault_list;

/**
 * @brief Internal panic function, required so we can override the default panic behavior and store data into the
 * watchdog
 *
 * This function is called by the assembly wrappers, which extract the caller of panic to store it in the watchdog.
 * Do not call this function directly! Use panic() instead
 */
void __attribute__((noreturn)) __printflike(1, 0) safety_panic_internal(const char *fmt, ...) {
    puts("\n*** PANIC ***\n");
    if (fmt) {
#if LIB_PICO_PRINTF_NONE
        puts(fmt);
#else
        va_list args;
        va_start(args, fmt);
#if PICO_PRINTF_ALWAYS_INCLUDED
        vprintf(fmt, args);
#else
        weak_raw_vprintf(fmt, args);
#endif
        va_end(args);
        puts("\n");
#endif
    }

    while (1) {
        // ========================================
        // ========== YOUR CODE CRASHED! ==========
        // ========================================
        // If you've gotten here, then your code panicked for some reason
        // This was a crash manually initiated by the code hitting a condition that cannot be recovered from
        // Check the debug uart and the stack trace to see what went wrong
        __breakpoint();
    }
}

// Holds the uptime so that it can be recovered
// We have around a 1 in a billion chance of random data corrupting this to match the XOR, so we should be fine
volatile uint32_t safety_uptime_ticks __attribute__((section(".uninitialized_data.safety_uptime_ticks")));
volatile uint32_t safety_uptime_ticks_xor __attribute__((section(".uninitialized_data.safety_uptime_ticks_xor")));

/**
 * @brief Simple systick handler which increments the uptime value in the watchdog scratch registers
 */
void safety_systick_handler(void) {
    // Note: Does not have any overflow handling
    // Also, this can technically be corrupted if the watchdog fires as this is ticking up, however that should
    // hopefully be pretty rare. But if uptime is constantly corrupted during watchdog resets, then this will be why
    safety_uptime_ticks++;
    safety_uptime_ticks_xor = UPTIME_TICK_XOR_MAGIC ^ safety_uptime_ticks;
}

// Watchdog reboot handling
extern void __real_watchdog_reboot(uint32_t pc, uint32_t sp, uint32_t delay_ms);

void __wrap_watchdog_reboot(uint32_t pc, uint32_t sp, uint32_t delay_ms) {
    *reset_reason_reg = CLEAN_BOOT;
    watchdog_hw->scratch[1] = CLEAN_RESET_TYPE_SOFTWARE;

    __real_watchdog_reboot(pc, sp, delay_ms);

    // The pico sdk watchdog reboot doesn't actually block, we'll do it since the caller probably assumes we won't
    // return
    do {
    } while (1);
}

// ========================================
// Crash Log Parsing
// ========================================

/**
 * @brief Formats the requested crash_log_entry into the char array passed into the function.
 *
 * @param entry The crash_log_entry to format
 * @param msg The buffer to write the formatted string to
 * @param size The size of `msg`
 */
static void safety_format_reset_cause_entry(struct crash_log_entry *entry, char *msg, size_t size) {
#define inc_safety_print(statement)                                                                                    \
    do {                                                                                                               \
        int ret = statement;                                                                                           \
        if (statement < 0)                                                                                             \
            return;                                                                                                    \
        if (ret >= (int) size)                                                                                         \
            return;                                                                                                    \
        size -= ret;                                                                                                   \
        msg += ret;                                                                                                    \
    } while (0)

    if (entry->reset_reason == CLEAN_BOOT) {
        const char *reset_type = "Invalid";
        if (entry->scratch_1 == CLEAN_RESET_TYPE_POR) {
            reset_type = "Power-on Reset";
        }
        else if (entry->scratch_1 == CLEAN_RESET_TYPE_RUN) {
            reset_type = "RESET Pin";
        }
        else if (entry->scratch_1 == CLEAN_RESET_TYPE_PSM) {
            reset_type = "PSM Debug Request";
        }
        else if (entry->scratch_1 == CLEAN_RESET_TYPE_SOFTWARE) {
            reset_type = "Software Request";
        }
        else if (entry->scratch_1 == CLEAN_RESET_TYPE_UNK_WDG) {
            reset_type = "Unknown Watchdog Request";
        }
        else if (entry->scratch_1 == CLEAN_RESET_TYPE_BOOTLOADER) {
            reset_type = "Bootloader";
        }
        else if (entry->scratch_1 == CLEAN_RESET_TYPE_BOOTROM) {
            reset_type = "Bootrom USB Upload";
        }

        inc_safety_print(snprintf(msg, size, "Clean Boot: %s", reset_type));
    }
    else {
        if (entry->reset_reason == UNKNOWN_SAFETY_PREINIT) {
            inc_safety_print(snprintf(msg, size, "UNKNOWN_SAFETY_PREINIT"));
        }
        else if (entry->reset_reason == UNKNOWN_SAFETY_ACTIVE) {
            inc_safety_print(snprintf(msg, size, "UNKNOWN_SAFETY_ACTIVE"));
        }
        else if (entry->reset_reason == PANIC) {
            inc_safety_print(snprintf(msg, size, "PANIC (Message: 0x%08lX, Call Address: 0x%08lX)", entry->scratch_1,
                                      entry->scratch_2));
        }
        else if (entry->reset_reason == HARD_FAULT) {
            inc_safety_print(snprintf(msg, size, "HARD_FAULT (Fault Address: 0x%08lX)", entry->scratch_1));
        }
        else if (entry->reset_reason == ASSERT_FAIL) {
            inc_safety_print(
                snprintf(msg, size, "ASSERT_FAIL (File: 0x%08lX Line: %ld)", entry->scratch_1, entry->scratch_2));
        }
        else if (entry->reset_reason == HARD_ASSERT) {
            inc_safety_print(snprintf(msg, size, "HARD_ASSERT (Call Address: 0x%08lX)", entry->scratch_1));
        }
        else if (entry->reset_reason == WATCHDOG_TIMEOUT) {
            inc_safety_print(snprintf(msg, size, "WATCHDOG_TIMEOUT (Last Address: 0x%08lX)", entry->scratch_1));
        }
        else if (entry->reset_reason == CORE1_TIMEOUT) {
            inc_safety_print(snprintf(msg, size, "CORE1_TIMEOUT (Last Address: 0x%08lX)", entry->scratch_1));
        }
        else {
            inc_safety_print(snprintf(msg, size, "Invalid Data in Reason Register"));
        }

        inc_safety_print(snprintf(msg, size, " - Faults: 0x%08lX", entry->faults));
    }

    // Uptime valid if non-zero
    if (entry->uptime) {
        float uptime_pretty;
        char uptime_units;
        if (entry->uptime > 3600 * uptime_ticks_per_sec) {
            uptime_pretty = (float) (entry->uptime) / (3600 * uptime_ticks_per_sec);
            uptime_units = 'h';
        }
        else if (entry->uptime > 60 * uptime_ticks_per_sec) {
            uptime_pretty = (float) (entry->uptime) / (60 * uptime_ticks_per_sec);
            uptime_units = 'm';
        }
        else {
            uptime_pretty = (float) (entry->uptime) / uptime_ticks_per_sec;
            uptime_units = 's';
        }

        inc_safety_print(snprintf(msg, size, " - Uptime: %.2f %c", uptime_pretty, uptime_units));
    }
}

/**
 * @brief Prints the last reset cause to the system log.
 */
static void safety_print_last_reset_cause(void) {
    if (crash_data.header.next_entry != 0 || crash_data.header.flags.log_wrapped) {
        char message_buf[256];
        message_buf[0] = '\0';

        int last_index =
            (crash_data.header.next_entry == 0 ? SAFETY_NUM_CRASH_LOG_ENTRIES - 1 : crash_data.header.next_entry - 1);
        struct crash_log_entry *last_reset = &crash_data.crash_log[last_index];

        char *msg = message_buf;
        size_t size = sizeof(message_buf);

        if (last_reset->reset_reason != CLEAN_BOOT) {
            inc_safety_print(
                snprintf(msg, size, "Watchdog Reset (Total Crashes: %d", crash_data.crash_counter.total_crashes));
            if (crash_data.crash_counter.panic_count) {
                inc_safety_print(snprintf(msg, size, " - Panics: %d", crash_data.crash_counter.panic_count));
            }
            if (crash_data.crash_counter.hard_fault_count) {
                inc_safety_print(snprintf(msg, size, " - Hard Faults: %d", crash_data.crash_counter.hard_fault_count));
            }
            if (crash_data.crash_counter.assert_fail_count) {
                inc_safety_print(
                    snprintf(msg, size, " - Assert Fails: %d", crash_data.crash_counter.assert_fail_count));
            }
            inc_safety_print(snprintf(msg, size, ") - "));
        }

        safety_format_reset_cause_entry(last_reset, msg, size);

        LOG_INFO("%s", message_buf);
    }
    else {
        LOG_WARN("No reset data found");
    }
}

/**
 * @brief Does processing of the last reset cause and prints it to serial.
 * Also performs the initialization of the crash_log if it is has not been initialized.
 *
 * Should only be called once and before any watchdog registers are overwritten.
 */
static void safety_process_last_reset_cause(void) {
    bool crash_data_needs_init = true;
    bool should_raise_fault = false;
    if (crash_data.header.flags.valid == VALID_MAGIC_VALUE) {
        uint16_t expected_crc = calc_crash_data_crc();
        if (expected_crc == crash_data.header.crc) {
            crash_data_needs_init = false;
        }
    }

    if (watchdog_enable_caused_reboot() && crash_data_needs_init) {
        LOG_ERROR("Crash log corrupted");
        should_raise_fault = true;
    }

    if (crash_data_needs_init) {
        memset(&crash_data, 0, sizeof(crash_data));
        crash_data.header.flags.valid = VALID_MAGIC_VALUE;
        crash_data.header.crc = calc_crash_data_crc();
    }

    // Handle data in watchdog registers
    struct crash_log_entry *next_entry = &crash_data.crash_log[crash_data.header.next_entry];

    // Pull uptime data if valid
    if (safety_uptime_ticks == (safety_uptime_ticks_xor ^ UPTIME_TICK_XOR_MAGIC)) {
        next_entry->uptime = safety_uptime_ticks;
    }
    else {
        next_entry->uptime = 0;
    }

    if (watchdog_enable_caused_reboot()) {
        // Extract data on watchdog reset
        next_entry->reset_reason = *reset_reason_reg;
        next_entry->faults = *fault_list_reg;
        next_entry->scratch_1 = watchdog_hw->scratch[1];
        next_entry->scratch_2 = watchdog_hw->scratch[2];

        if (crash_data.crash_counter.total_crashes != 0xFF)
            crash_data.crash_counter.total_crashes++;
        if (next_entry->reset_reason == PANIC && crash_data.crash_counter.panic_count != 0xFF)
            crash_data.crash_counter.panic_count++;
        if (next_entry->reset_reason == HARD_FAULT && crash_data.crash_counter.hard_fault_count != 0xFF)
            crash_data.crash_counter.hard_fault_count++;
        if (next_entry->reset_reason == ASSERT_FAIL && crash_data.crash_counter.assert_fail_count != 0xFF)
            crash_data.crash_counter.assert_fail_count++;
        if (next_entry->reset_reason == HARD_ASSERT && crash_data.crash_counter.assert_fail_count != 0xFF)
            crash_data.crash_counter.assert_fail_count++;

        if (next_entry->reset_reason == CLEAN_BOOT) {
            LOG_ERROR("Clean boot specified in reset_reason register, but the watchdoog has reported a reset");
        }

        should_raise_fault = true;

        // Dump profiler data from last session
        profiler_dump();
    }
    else {
        // Clear all crash counters to clear sticky fault
        crash_data.crash_counter.i = 0;

        // Decode the clean boot reset cause and write to log
        next_entry->reset_reason = CLEAN_BOOT;
        if (*reset_reason_reg == CLEAN_BOOT) {
            next_entry->scratch_1 = watchdog_hw->scratch[1];
            watchdog_hw->scratch[1] = 0;
        }
        else {
            next_entry->scratch_1 = 0;
            if (watchdog_caused_reboot()) {
                next_entry->scratch_1 = CLEAN_RESET_TYPE_UNK_WDG;
            }
            else {
                if (vreg_and_chip_reset_hw->chip_reset & VREG_AND_CHIP_RESET_CHIP_RESET_HAD_POR_BITS) {
                    next_entry->scratch_1 = CLEAN_RESET_TYPE_POR;
                }
                if (vreg_and_chip_reset_hw->chip_reset & VREG_AND_CHIP_RESET_CHIP_RESET_HAD_RUN_BITS) {
                    next_entry->scratch_1 = CLEAN_RESET_TYPE_RUN;
                }
                if (vreg_and_chip_reset_hw->chip_reset & VREG_AND_CHIP_RESET_CHIP_RESET_HAD_PSM_RESTART_BITS) {
                    next_entry->scratch_1 = CLEAN_RESET_TYPE_PSM;
                }
            }
        }
    }

    // Clear all watchdog registers that contain state for current reboot session
    safety_uptime_ticks = 1;  // Setting to 1 since 0 is invalid (and it's already a rough estimate anyways)
    safety_uptime_ticks_xor = safety_uptime_ticks ^ UPTIME_TICK_XOR_MAGIC;
    *fault_list_reg = 0;
    profiler_reset(false);

    // Apply changes to next_entry and checksum
    if (crash_data.header.next_entry == (SAFETY_NUM_CRASH_LOG_ENTRIES - 1)) {
        crash_data.header.next_entry = 0;
        crash_data.header.flags.log_wrapped = 1;
    }
    else {
        crash_data.header.next_entry += 1;
    }
    crash_data.header.crc = calc_crash_data_crc();

    // Perform all operations which print data to the console
    safety_print_last_reset_cause();
    if (should_raise_fault) {
        safety_raise_fault(FAULT_WATCHDOG_RESET);
    }
}

// ========================================
// Public Functions
// ========================================

void safety_internal_crash_reporting_handle_reset(void) {
    safety_process_last_reset_cause();

    // Claim spinlock for fault handling
    spin_lock_claim(SAFETY_CRASH_LOGGED_SPINLOCK_ID);

    // Set hardfault handler
    exception_set_exclusive_handler(HARDFAULT_EXCEPTION, &safety_hard_fault_handler);

    // Enable systick for uptime counting
    exception_set_exclusive_handler(SYSTICK_EXCEPTION, &safety_systick_handler);
    // Configuring systick reload value. Reference clock is a 1 us pulse
    systick_hw->rvr = (1000000 / uptime_ticks_per_sec) - 1;
    static_assert(1000000 % uptime_ticks_per_sec == 0,
                  "Uptime ticks per second must be evenly divide into microseconds");
    // Set systick to start at reload value
    systick_hw->cvr = systick_hw->rvr;
    // Clear COUNTFLAG by reading CSR
    systick_hw->csr;
    // Enable systick with SysTick exception enabled using external reference clock
    systick_hw->csr = M0PLUS_SYST_CSR_TICKINT_BITS | M0PLUS_SYST_CSR_ENABLE_BITS;

    // Set reset reason
    *reset_reason_reg = UNKNOWN_SAFETY_PREINIT;
}

void safety_internal_crash_reporting_handle_init(void) {
    *reset_reason_reg = UNKNOWN_SAFETY_ACTIVE;
}

void safety_internal_crash_reporting_handle_deinit(void) {
    *reset_reason_reg = UNKNOWN_SAFETY_PREINIT;
}

void safety_print_crash_log(void) {
    hard_assert_if(SAFETY, !safety_is_setup);

    char message_buf[256];
    message_buf[0] = '\0';

    char *msg = message_buf;
    size_t size = sizeof(message_buf);

    LOG_INFO("===========Watchdog Crash Log===========");
    inc_safety_print(snprintf(msg, size, "Total Crashes: %d", crash_data.crash_counter.total_crashes));
    if (crash_data.crash_counter.panic_count) {
        inc_safety_print(snprintf(msg, size, " - Panics: %d", crash_data.crash_counter.panic_count));
    }
    if (crash_data.crash_counter.hard_fault_count) {
        inc_safety_print(snprintf(msg, size, " - Hard Faults: %d", crash_data.crash_counter.hard_fault_count));
    }
    if (crash_data.crash_counter.assert_fail_count) {
        inc_safety_print(snprintf(msg, size, " - Assert Fails: %d", crash_data.crash_counter.assert_fail_count));
    }
    LOG_INFO("%s", message_buf);

    int i = crash_data.header.next_entry;
    __unused int count = 1;
    do {
        if (i == 0) {
            if (crash_data.header.flags.log_wrapped) {
                i = SAFETY_NUM_CRASH_LOG_ENTRIES;
            }
            else {
                break;
            }
        }
        i--;

        safety_format_reset_cause_entry(&crash_data.crash_log[i], message_buf, sizeof(message_buf));
        LOG_INFO("Reset # %3d: %s", -(count++), message_buf);
    } while (i != crash_data.header.next_entry);

    LOG_INFO("========================================");
}

void safety_enter_bootloader(void) {
    // Set proper fields to be decoded after exiting bootloader mode
    *reset_reason_reg = CLEAN_BOOT;
    watchdog_hw->scratch[1] = CLEAN_RESET_TYPE_BOOTLOADER;

    // Set the flag to tell bootloader to start rather than user application
    watchdog_hw->scratch[4] = 0xb00710ad;

    // Disable current watchdog
    hw_clear_bits(&watchdog_hw->ctrl, WATCHDOG_CTRL_ENABLE_BITS);

    // Reset everything except oscillators
    hw_set_bits(&psm_hw->wdsel, PSM_WDSEL_BITS & ~(PSM_WDSEL_ROSC_BITS | PSM_WDSEL_XOSC_BITS));

    // Trigger reset now
    hw_set_bits(&watchdog_hw->ctrl, WATCHDOG_CTRL_TRIGGER_BITS);

    // Halt
    while (1) {
        tight_loop_contents();
    }
}
