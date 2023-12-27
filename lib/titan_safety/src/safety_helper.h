#ifndef SAFETY_HELPER_H_
#define SAFETY_HELPER_H_

/**
 * @brief This is the id of the spinlock to use for ensuring only *one* crash is logged to the fault register
 *
 * In the event multiple crashes occur (on either core), only the first one will be logged.
 * This is done by keeping the spin lock unlocked until a crash occurs. Then, before writing the watchdog scratch
 * registers, this spin lock will be acquired and never released.
 *
 * Any further crash calls that try to acquire this lock will fail, and they won't write to the watchdog regs.
 * This will be kept this way until the MCU is reset (and when runtime_init is called on next start, all spin locks are
 * released)
 *
 * @note This should be the same as PICO_SPINLOCK_ID_OS1, however this header must be compatible with assembly
 */
#define SAFETY_CRASH_LOGGED_SPINLOCK_ID 14

// PICO_CONFIG: SAFETY_WATCHDOG_ALARM_NUM, Selects the hardware alarm to use for capturing the program counter right before reset. Set to -1 to disable, type=int, default=2, group=titan_safety
#ifndef SAFETY_WATCHDOG_ALARM_NUM
#define SAFETY_WATCHDOG_ALARM_NUM 2
#endif

/*
 * List of external functions defined in safety_helpder
 */
#ifndef __ASSEMBLER__
#include <stdbool.h>

extern void safety_hard_fault_handler(void);
extern void safety_nmi_handler(void);
extern void safety_halt_other_core(void);

extern bool core1_nmi_capture_addr;
#endif

/*
 * List of exports to the assembly
 * They additionally have the c declarations to make sure our C code is right
 */
#ifdef __ASSEMBLER__
// clang-format off
.extern safety_multicore_running
.extern safety_panic_internal
// clang-format on

#else
extern bool safety_multicore_running;
void __attribute__((noreturn)) safety_panic_internal(const char *fmt, ...);
#endif

#endif
