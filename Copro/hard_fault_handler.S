#include "hardware/regs/addressmap.h"
#include "hardware/regs/watchdog.h"

#define HARD_FAULT_DATA 0x1035004

// Custom HardFault Exception Handler
// Logs details of the fault to scratch registers for easier debugging
// See safety.c Watchdog Crash Reporting Functions for more details on format

.weak safety_hard_fault_handler
.type safety_hard_fault_handler,%function
.thumb_func
safety_hard_fault_handler:
    // Set the fault type in watchdog scratch 0
    ldr r0, =(HARD_FAULT_DATA)
    ldr r1, =(WATCHDOG_BASE + WATCHDOG_SCRATCH0_OFFSET)
    str r0, [r1]

    // Increment hard fault counter in sratch 3
    ldr r1, =(WATCHDOG_BASE + WATCHDOG_SCRATCH3_OFFSET)
    ldr r0, [r1]

    // Don't increment if already maxed out
    mov r2, #0xFF
    lsl r2, r2, #0x10
    orr r2, r2, r0
    cmp r0, r2
    beq _skip_add

    // Add 0x010000 to the watchdog scratch 3 register
    mov r2, #1
    lsl r2, r2, #0x10
    add r0, r0, r2
    str r0, [r1]
_skip_add:

    // Save the program counter of the hard fault
    ldr r1, =(WATCHDOG_BASE + WATCHDOG_SCRATCH1_OFFSET)
    // Save #0xFFFFFFFF into the register in case msp is corrupted
    mov r0, #0
    sub r0, r0, #1
    str r0, [r1]
    // Try to load program counter
    ldr r0, [sp, #0x18]
    str r0, [r1]

    bkpt #0
