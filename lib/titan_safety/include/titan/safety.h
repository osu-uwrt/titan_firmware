#ifndef TITAN__SAFETY_H_
#define TITAN__SAFETY_H_

#include "pico/time.h"

#include <stdbool.h>
#include <stdint.h>

/**
 * @file safety.h
 *
 * API for managing firmware safety/reliability related functions.
 *
 * This library has five main components
 *  - Fault Reporting: Implements a fault tracking system in a 32-bit ID. If a fault occurs, a bit will be set
 *      corresponding to a fault ID defined by the implementer in the fault list. A callback will be raised to indicate
 *      to the user that a fault has occurred (such as an LED lighting up).
 *
 *  - Kill Switch: Implements a set of functions to allow multiple kill switches to be assigned, which can be confiugred
 *      to require a constant update to remain enabled. Upon kill switch activation, an implementation defined callback
 *      allows the implementer to immediately respond to a kill switch request. Functions are also implemented to check
 *      the current kill switch state, and be notified when the system is enabled
 *
 *  - Crash Reporting: Hooks various parts of the low-level firmware to provide in-depth information about any crashes
 *      in the system to aid in debugging. This code hooks the systick handler to report an uptime, the hardfault,
 *      assertion, and panic handlers to provide crash information, and stores this information for
 *      SAFETY_NUM_CRASH_LOG_ENTRIES.
 *
 *  - Profiler: Records timestamps of various critical points in code to determine execution time for the various
 *      functions in firmware. This data is recorded and can be printed during a watchdoog reset and extracted during
 *      runtime.
 *
 *  - Watchdog Timer: All of these functions are integrated with the chip's watchdog timer such that critical code,
 *      such as kill switch updates, are never missed.
 */

// PICO_CONFIG: SAFETY_KILL_SWITCH_TIMEOUT_MS, Timeout in milliseconds of kill switch commands requiring updating. Note it may take longer to disable as it is only refreshed during safety_tick, type=int, default=500, group=titan_safety
#ifndef SAFETY_KILL_SWITCH_TIMEOUT_MS
#define SAFETY_KILL_SWITCH_TIMEOUT_MS 500
#endif

// PICO_CONFIG: SAFETY_SOFTWARE_KILL_FRAME_STR_SIZE, Maximum size of software kill frame string to avoid competing software kill publishers, type=int, default=32, group=titan_safety
#ifndef SAFETY_SOFTWARE_KILL_FRAME_STR_SIZE
#define SAFETY_SOFTWARE_KILL_FRAME_STR_SIZE 32
#endif

// PICO_CONFIG: SAFETY_NUM_CRASH_LOG_ENTRIES, Number of crash log entries to store in crash history, type=int, default=24, group=titan_safety
#ifndef SAFETY_NUM_CRASH_LOG_ENTRIES
#define SAFETY_NUM_CRASH_LOG_ENTRIES 24
#endif

// ========================================
// Fault Management Functions
// ========================================

// MAX_FAULT_ID is determined by the fault_list_reg size (32-bit)
#define MAX_FAULT_ID 31

// XMACRO to define the libsafety specific faults (watchdog reset and watchdog warning)
// The XMACRO follows the form DO(name, val) where name is the fault name (should begin with FAULT_)
// and val is the ID for that fault. The fault id must be less than MAX_FAULT_ID
#define XLIST_OF_LIBSAFETY_FAULTS(DEF)                                                                                 \
    DEF(FAULT_WATCHDOG_RESET, 0)                                                                                       \
    DEF(FAULT_WATCHDOG_WARNING, 1)
// All other fault ids are implentation specific

struct fault_data {
    absolute_time_t time;  // The time this fault mot recently occurred
    uint32_t extra_data;   // Extra data from the most recent firing of this fault
    const char *filename;  // The filename where this fault most recently occurred
    uint16_t line;         // The line where this fault most recently occurred
    bool multiple_fires;   // Set if fault raised multiple times
    bool sticky_fault;     // Set on raised, but not cleared by lower, can be used to see if other data in this is valid
};

/**
 * @brief Array of fault_data for every fault.
 * Contains MAX_FAULT_ID+1 entries.
 *
 * @attention Do not write to this variable
 */
extern struct fault_data safety_fault_data[];

/**
 * @brief A pointer to the list of all the active faults as bits
 *
 * @attention Do not write to this variable, use safety_raise_fault/safety_lower_fault instead
 */
extern volatile uint32_t *const fault_list_reg;

/**
 * @brief Raises the specified fault id. Note the macros should be used to autopopulate
 * filename and line.
 *
 * @note Safe to be called from any context, including interrupts and on any core.
 *
 * @param fault_id The id to be raised. Faults are defined above
 * @param arg Additional data to store alongside fault information
 * @param filename The file the fault was raised
 * @param line The line the fault was raised
 */
void safety_raise_fault_full(uint32_t fault_id, uint32_t arg, const char *filename, uint16_t line);

/**
 * @brief Raises the specific fault id.
 *
 * @note Safe to be called from any context, including interrupts and on any core.
 *
 * @param fault_id The fault id to be raised. Faults are defined above
 */
#define safety_raise_fault(fault_id) safety_raise_fault_full(fault_id, 0, __FILE__, __LINE__)

/**
 * @brief Raises the specific fault id.
 *
 * @note Safe to be called from any context, including interrupts and on any core.
 *
 * @param fault_id The fault id to be raised. Faults are defined above
 * @param arg Additional data to hold alongside fault information
 */
#define safety_raise_fault_with_arg(fault_id, arg)                                                                     \
    safety_raise_fault_full(fault_id, (uint32_t) (arg), __FILE__, __LINE__)

/**
 * @brief Lowers the specified fault id
 *
 * @note Safe to be called from any context, including interrupts and on any core.
 *
 * @param fault_id The id to be lowered. Faults are defined above
 */
void safety_lower_fault(uint32_t fault_id);

/**
 * @brief Looks up fault id name for a given fault id
 *
 * @note Safe to be called from any context, including interrupts and on any core.
 *
 * @param fault_id The fault id to lookup
 * @return const char* The fault name
 */
const char *safety_lookup_fault_id(uint32_t fault_id);

// ========================================
// Kill Switch Management Functions
// ========================================

struct kill_switch_state {
    bool enabled;                    // If the specific kill switch is enabled
    bool asserting_kill;             // If the kill switch is asserting a kill request (system disable)
    bool needs_update;               // If the switch needs to be updated or it will be considered killed
    absolute_time_t update_timeout;  // The last update timestamp of the switch

    // The frame that asserted the kill switch
    // Prevents another node from de-asserting kill by publishing that it is not killed with the same switch id
    char locking_frame[SAFETY_SOFTWARE_KILL_FRAME_STR_SIZE];
};

/**
 * @brief State of all of the kill switches for reading
 *
 * @attention Do not write to this variable, use safety_kill_switch_update instead
 */
extern struct kill_switch_state kill_switch_states[];

/**
 * @brief Updates a kill switch state
 *
 * @note Safe to be called from interrupts.
 *
 * @attention This must only be called on core 0
 *
 * @param switch_num The unique number for that kill switch. MUST BE < MAX_KILL_SWITCHES
 * @param asserting_kill If the kill switch is asserting a kill request (system disable)
 * @param needs_update Setting to true will require the kill switch to be updated within SAFETY_KILL_SWITCH_TIMEOUT_MS
 * or else will assert kill
 */
void safety_kill_switch_update(uint8_t switch_num, bool asserting_kill, bool needs_update);

/**
 * @brief Returns if the kill switch is asserting a safety kill
 *
 * @note Safe to be called from any context, including interrupts and on any core.
 *
 * @attention Ensure that this value cannot be raced between the reading of the current value, and any events which
 * can be killed, as kill switch events may fire between reading this value and before the critical function.
 *
 * @return true  Any action requiring a kill switch insertion should not run
 * @return false All kill switches are reporting okay and the operation can run
 */
bool safety_kill_get_asserting_kill(void);

/**
 * @brief Gets the time of the last kill switch change
 *
 * @note Safe to be called from any context, including interrupts and on any core.
 *
 * @return absolute_time_t The time of last kill switch change as an absolute time
 */
absolute_time_t safety_kill_get_last_change(void);

// ========================================
// Safety Watchdog-Related Functions
// ========================================

struct crash_data {
    union {
        uint32_t i;
        struct __attribute__((__packed__)) {
            uint16_t crc;
            union {
                uint8_t i;
                struct {
#define VALID_MAGIC_VALUE 0x5
                    uint8_t valid:4;
                    uint8_t log_wrapped:1;
                };
            } flags;
            uint8_t next_entry;
        };
    } header;

    union {
        uint32_t i;
        struct __attribute__((__packed__)) {
            uint8_t total_crashes;
            uint8_t panic_count;
            uint8_t hard_fault_count;
            uint8_t assert_fail_count;
        };
    } crash_counter;

    struct crash_log_entry {
        uint32_t reset_reason;
        uint32_t scratch_1;
        uint32_t scratch_2;
        uint32_t uptime;

        uint32_t faults;
    } crash_log[SAFETY_NUM_CRASH_LOG_ENTRIES];
};

extern struct crash_data crash_data;

/**
 * @brief Resets into bootloader mode.
 * This issues a watchdog reset with the proper flags set to enter bootloader mode.
 *
 * @attention This must only be called on core 0
 *
 * @note This function does not return
 */
void safety_enter_bootloader(void);

/**
 * @brief Prints the full watchdog crash log to the system log
 *
 * @note safety_setup must be called before this function can be used
 */
void safety_print_crash_log(void);

// ========================================
// Safety Limetime Functions
// ========================================

/**
 * @brief Boolean for if safety_setup has been called
 */
extern bool safety_is_setup;

/**
 * @brief Boolean for if safety_init has been called
 */
extern bool safety_initialized;

/**
 * @brief Performs core safety setup to be completed immediately after reset
 * This enables a watchdog timer, but with a multi-second long tick for setup
 * Will also print data on the last reset cause and any crash data from that
 *
 * @note Can only be called once
 *
 * @attention This function MUST be called before any other safety functions. Failure to do so may result in corruption
 * or loss of current or previous boot's safety state
 */
void safety_setup(void);

/**
 * @brief Initializes safety for normal operation
 * This will tighten the timing for the watchdog timer, and is required for the kill switch to be enabled
 *
 * @attention Do not call again without calling safety_deinit
 */
void safety_init(void);

/**
 * @brief Deinitializes safety for setup
 * This will loosen the timing for the watchdog timer.
 *
 * @attention safety_init must be called before this function can be used
 */
void safety_deinit(void);

/**
 * @brief Ticks safety
 * This must be called within the period of the watchdog timer or a reset will occur
 *
 */
void safety_tick(void);

// ========================================
// Safety Multicore Functions
// ========================================

/**
 * @brief Launches core 1 with safety bindings.
 * This function will launch core 1, and then ensure that core 1 checks in every SAFETY_CORE1_CHECKIN_INTERVAL_MS.
 *
 * @attention Can only be called once
 *
 * @param entry The entry point for core 1
 */
void safety_launch_core1(void (*entry)(void));

/**
 * @brief Checks in with safety to notify that core 1 is still alive. This must be called of an interval of at least
 * SAFETY_CORE1_CHECKIN_INTERVAL_MS on core 1.
 *
 * @attention Can only be called on core 1 and after safety_launch_core1 is called.
 */
void safety_core1_checkin(void);

// ========================================
// Profiler Functions
// ========================================

// WIP - Sort of works, but not fully yet. See puddles firmware for current state

#ifndef SAFETY_ENABLE_PROFILER
#define SAFETY_ENABLE_PROFILER 0
#endif

#if SAFETY_ENABLE_PROFILER

void profiler_push(uint32_t profiler_id);
void profiler_pop(uint32_t profiler_id);
void profiler_reset(bool check_not_popped);
void profiler_dump(void);

#else

static inline void profiler_push(__unused uint32_t profiler_id) {}
static inline void profiler_pop(__unused uint32_t profiler_id) {}
static inline void profiler_reset(__unused bool check_not_popped) {}
static inline void profiler_dump(void) {}

#endif

#endif
