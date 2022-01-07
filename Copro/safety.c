#include <stdarg.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdint.h>

#include "pico/stdlib.h"
#include "hardware/exception.h"
#include "hardware/structs/watchdog.h"
#include "hardware/structs/psm.h"
#include "hardware/sync.h"
#include "hardware/watchdog.h"

#include "dio.h"
#include "dshot.h"
#include "safety.h"

// ========================================
// Fault Management Functions
// ========================================

uint32_t fault_list = 0;

void safety_raise_fault(uint32_t fault_id) {
    valid_params_if(SAFETY, fault_id <= MAX_FAULT_ID);

    if ((fault_list & (1u<<fault_id)) == 0) {
        printf("Fault %d Raised\n", fault_id);

        // To ensure the fault led doesn't get glitched on/off due to an untimely interrupt, interrupts will be disabled during
        // the setting of the fault state and the fault LED

        uint32_t prev_interrupt_state = save_and_disable_interrupts();
        
        fault_list |= (1<<fault_id);
        dio_set_fault_led(true);
        
        restore_interrupts(prev_interrupt_state);
    }
}

void safety_lower_fault(uint32_t fault_id) {
    valid_params_if(SAFETY, fault_id <= MAX_FAULT_ID);

    if ((fault_list & (1u<<fault_id)) != 0) {
        printf("Fault %d Lowered\n", fault_id);
        
        // To ensure the fault led doesn't get glitched on/off due to an untimely interrupt, interrupts will be disabled during
        // the setting of the fault state and the fault LED

        uint32_t prev_interrupt_state = save_and_disable_interrupts();
        
        fault_list &= ~(1u<<fault_id);
        dio_set_fault_led(fault_list != 0);
        
        restore_interrupts(prev_interrupt_state);
    }
}



// ========================================
// Kill Switch Management Functions
// ========================================

static absolute_time_t last_kill_switch_change;
static bool last_state_asserting_kill = true; // Start asserting kill
struct kill_switch_state kill_switch_states[MAX_KILL_SWITCHES] = {[0 ... MAX_KILL_SWITCHES-1] = { .enabled = false }};

/**
 * @brief Contains all callback functions to be called when robot is killed
 * Called from any function that will kill the robot
 * 
 * This function should only be called when safety is initialized
 */
static void safety_kill_robot(void) {
    last_state_asserting_kill = true;
    last_kill_switch_change = get_absolute_time();

    // Note: Any calls made in this function must be safe to be called from interrupts
    // This is because safety_kill_switch_update can be called from interrupts

    // TODO: Implement kill notifier
    dshot_stop_thrusters();
}

/**
 * @brief Refreshes kill switches to check for any timeouts
 * It is responsible for re-enabling the robot after all of the kill switches have been released.
 * 
 * This function should only be called when safety is initialized
 */
static void safety_refresh_kill_switches(void) {
    absolute_time_t now = get_absolute_time();

    // Check all kill switches for asserting kill
    bool asserting_kill = false;
    int num_switches_enabled = 0;
    for (int i = 0; i < MAX_KILL_SWITCHES; i++) {
        if (kill_switch_states[i].enabled) {
            num_switches_enabled++;

            // Kill if asserting kill or if timeout expired when requiring update
            if (kill_switch_states[i].asserting_kill || 
                    (kill_switch_states[i].needs_update && absolute_time_diff_us(now, kill_switch_states[i].update_timeout) < 0)) {
                asserting_kill = true;
                break;
            }
        }
    }

    // If no kill switches are enabled, force into kill as a precaution
    if (num_switches_enabled == 0) {
        asserting_kill = true;
    }

    // Update last state, and notify of kill if needed
    if (last_state_asserting_kill != asserting_kill) {
        last_state_asserting_kill = asserting_kill;

        if (asserting_kill) {
            safety_kill_robot();
        } else {
            last_kill_switch_change = get_absolute_time();
        }
    }
}

void safety_kill_switch_update(uint8_t switch_num, bool asserting_kill, bool needs_update){
    valid_params_if(SAFETY, switch_num < MAX_KILL_SWITCHES);

    kill_switch_states[switch_num].asserting_kill = asserting_kill;
    kill_switch_states[switch_num].update_timeout = make_timeout_time_ms(KILL_SWITCH_TIMEOUT_MS);
    kill_switch_states[switch_num].needs_update = needs_update;
    kill_switch_states[switch_num].enabled = true;

    if (safety_initialized && asserting_kill) {
        safety_kill_robot();
    }
}

bool safety_kill_get_asserting_kill(void) {
    hard_assert_if(LIFETIME_CHECK, !safety_initialized);
    return last_state_asserting_kill;
}

absolute_time_t safety_kill_get_last_change(void) {
    hard_assert_if(LIFETIME_CHECK, !safety_initialized);
    return last_kill_switch_change;
}



// ========================================
// Watchdog Crash Reporting Functions
// ========================================

// Use of Watchdog Scratch Registers
// Can really only be considered valid if 
// scratch[0]: Last Crash Action
//  - UNKNOWN_SAFETY_PREINIT: Unknown, crashed after safety_setup
//  - UNKNOWN_SAFETY_ACTIVE: Unknown, crashed after safety_init
//  - PANIC
//     scratch[1]: Address of panic string (Won't be dereferenced for safety)
//  - HARD_FAULT
//     scratch[1]: Faulting Address
// scratch[3]: Watchdog Reset Counter (LSB order)
//     Byte 0: Total Watchdog Reset Counter
//     Byte 1: Panic Reset Counter
//     Byte 2: Hard Fault Reset Counter
//     Byte 3: Assertion Fail Reset Counter
#define UNKNOWN_SAFETY_PREINIT  0x1035001
#define UNKNOWN_SAFETY_ACTIVE   0x1035002
#define PANIC                   0x1035003
#define HARD_FAULT              0x1035004
#define ASSERT_FAIL             0x1035005

// Very useful for debugging, prevents cpu from resetting while execution is halted for debugging
// However, this should be ideally be disabled when not debugging in the event something goes horribly wrong
#define PAUSE_WATCHDOG_ON_DEBUG 1

static volatile uint32_t *reset_reason_reg = &watchdog_hw->scratch[0];

// Defined in hard_fault_handler.S
extern void safety_hard_fault_handler(void);
static exception_handler_t original_hardfault_handler = NULL;

// Assertion Handling
extern void __real___assert_func(const char *file, int line, const char *func, const char *failedexpr);

void __wrap___assert_func(const char *file, int line, const char *func, const char *failedexpr) {
    *reset_reason_reg = ASSERT_FAIL;
    watchdog_hw->scratch[1] = (uint32_t) file;
    watchdog_hw->scratch[2] = (uint32_t) line;
    watchdog_hw->scratch[6] = (uint32_t) func;
    watchdog_hw->scratch[7] = (uint32_t) failedexpr;
    if ((watchdog_hw->scratch[3] & 0xFF000000) != 0xFF000000) {
        watchdog_hw->scratch[3] += 0x1000000;
    }

    // Remove the hard fault exception handler so it doesn't overwrite panic data when the breakpoint is hit
    if (original_hardfault_handler != NULL) {
        exception_restore_handler(HARDFAULT_EXCEPTION, original_hardfault_handler);
    }
    
    __real___assert_func(file, line, func, failedexpr);
}

/**
 * @brief Custom panic handler to save the panic in watchdog scratch registers
 * 
 * SHOULD NOT BE CALLED DIRECTLY. Use `panic` instead
 * 
 * @param fmt printf like format string for panic
 * @param ... Args for format string
 */
void safety_panic(const char *fmt, ...) {
    *reset_reason_reg = PANIC;
    watchdog_hw->scratch[1] = (uint32_t) fmt;
    if ((watchdog_hw->scratch[3] & 0xFF00) != 0xFF00) {
        watchdog_hw->scratch[3] += 0x100;
    }

    puts("\n*** PANIC ***\n");

    if (fmt) {
        va_list args;
        va_start(args, fmt);
        vprintf(fmt, args);
        va_end(args);
        puts("\n");
    }

    // Remove the hard fault exception handler so it doesn't overwrite panic data when the breakpoint is hit
    if (original_hardfault_handler != NULL) {
        exception_restore_handler(HARDFAULT_EXCEPTION, original_hardfault_handler);
    }

    __breakpoint();
}

/**
 * @brief Does processing of the last reset cause and prints it to serial.
 * Also increments the total watchdog reset counter
 * 
 * Should only be called once and before reset reason is overwritten
 */
static void safety_process_last_reset_cause(void) {
    // Print last state
    if (watchdog_enable_caused_reboot()) {
        if ((watchdog_hw->scratch[3] & 0xFF) != 0xFF) watchdog_hw->scratch[3]++;

        printf("Watchdog Reset (Counts: 0x%x) - Reason: ", watchdog_hw->scratch[3]);

        uint32_t reset_reason = *reset_reason_reg;
        if (reset_reason == UNKNOWN_SAFETY_PREINIT) {
            printf("UNKNOWN_SAFETY_PREINIT\n");
        } else if (reset_reason == UNKNOWN_SAFETY_ACTIVE) {
            printf("UNKNOWN_SAFETY_ACTIVE\n");
        } else if (reset_reason == PANIC) {
            printf("PANIC (Message: 0x%08x)\n", watchdog_hw->scratch[1]);
        } else if (reset_reason == HARD_FAULT) {
            printf("HARD_FAULT (Fault Address: 0x%08x)\n", watchdog_hw->scratch[1]);
        } else if (reset_reason == ASSERT_FAIL) {
            printf("ASSERT_FAIL (File: 0x%08x Line: %d - Function: 0x%08x - Expression: 0x%08x)\n", watchdog_hw->scratch[1], watchdog_hw->scratch[2], watchdog_hw->scratch[6], watchdog_hw->scratch[7]);
        } else {
            printf("Invalid Data in Reason Register");
        }

        safety_raise_fault(FAULT_WATCHDOG_RESET);
    } else {
        watchdog_hw->scratch[3] = 0;
        printf("Clean boot\n");
    }
}

// ========================================
// Safety Limetime Functions
// ========================================

bool safety_initialized = false;
bool safety_is_setup = false;

void safety_setup(void) {
    hard_assert_if(LIFETIME_CHECK, safety_setup || safety_initialized);

    // Set hardfault handler
    original_hardfault_handler = exception_set_exclusive_handler(HARDFAULT_EXCEPTION, &safety_hard_fault_handler);

    safety_process_last_reset_cause();

    // Set reset reason
    *reset_reason_reg = UNKNOWN_SAFETY_PREINIT;
    safety_is_setup = true;

    // Enable slow watchdog while connecting
    watchdog_enable(3000, PAUSE_WATCHDOG_ON_DEBUG);
}

void safety_init(void) {
    hard_assert_if(LIFETIME_CHECK, !safety_setup || safety_initialized);

    safety_initialized = true;
    *reset_reason_reg = UNKNOWN_SAFETY_ACTIVE;

    // Populate the last kill switch change time to when safety is set up
    last_kill_switch_change = get_absolute_time();

    // Set tight watchdog timer for normal operation
    watchdog_enable(250, PAUSE_WATCHDOG_ON_DEBUG);
}

void safety_tick(void) {
    hard_assert_if(LIFETIME_CHECK, !safety_setup);

    // Check for any kill switch timeouts
    if (safety_initialized) {
        safety_refresh_kill_switches();
    }

    watchdog_update();
}