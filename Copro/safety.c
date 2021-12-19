#include <stdarg.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdint.h>

#include "hardware/watchdog.h"
#include "hardware/structs/watchdog.h"
#include "hardware/structs/psm.h"

#include "safety.h"

// Use of Watchdog Scratch Registers
// Can really only be considered valid if 
// scratch[0]: Last Crash Action
//  - UNKNOWN_SAFETY_PREINIT: Unknown, crashed after safety_setup
//  - UNKNOWN_SAFETY_ACTIVE: Unknown, crashed after safety_init
//  - PANIC
//     scratch[1]: Address of panic string (Won't be dereferenced for safety)
//  - HARD_FAULT
//     scratch[1]: Fault Type
//     scratch[2]: Faulting Address
// scratch[3]: Watchdog Reset Counter
//     Byte 0: Total Watchdog Reset Counter
//     Byte 1: Panic Reset Counter
//     Byte 2: Hard Fault Reset Counter
#define UNKNOWN_SAFETY_PREINIT  0x1035001
#define UNKNOWN_SAFETY_ACTIVE   0x1035002
#define PANIC                   0x1035003
#define HARD_FAULT              0x1035004


// Very useful for debugging, prevents cpu from resetting while interrupted
// However, this should be ideally be disabled when not debugging in the event something goes horribly wrong
#define PAUSE_WATCHDOG_ON_DEBUG 1

bool safety_initialized = false;
static volatile uint32_t *reset_reason_reg = &watchdog_hw->scratch[0];

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
}

void safety_setup(void) {
    if (safety_initialized) {
        panic("Safety already initialized when safety setup called");
    }

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
            printf("HARD_FAULT\n");
        } else {
            printf("Invalid Data in Reason Register");
        }
    } else {
        watchdog_hw->scratch[3] = 0;
        printf("Clean boot\n");
    }

    // Set reset reason
    *reset_reason_reg = UNKNOWN_SAFETY_PREINIT;

    // Enable slow watchdog while connecting
    watchdog_enable(3000, PAUSE_WATCHDOG_ON_DEBUG);
}

void safety_init(void) {
    if (safety_initialized) {
        panic("Safety already initialized when safety init called");
    }

    safety_initialized = true;
    *reset_reason_reg = UNKNOWN_SAFETY_ACTIVE;

    // Set tight watchdog timer for normal operation
    watchdog_enable(250, PAUSE_WATCHDOG_ON_DEBUG);
}

void safety_tick(void) {
    watchdog_update();
}