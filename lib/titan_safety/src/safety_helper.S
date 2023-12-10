#include "hardware/regs/addressmap.h"
#include "hardware/regs/watchdog.h"
#include "pico/asm_helper.S"
#include "titan/safety_magic_values.h"

pico_default_asm_setup

.text

// Safety panic wrapper
// This extracts the calling address, as well as the panic message, and saves it to the watchdog
// Then calls the internal panic function which will print and halt the CPU accordingly

.extern safety_panic_internal

.global safety_panic
.type safety_panic,%function
.thumb_func
safety_panic:
    // Note this is done in assembly so we can pull the lr out of the code
    // Additionally, we don't want to pass this data to the C function so we don't mess up the varargs
    // Because of this, we get this lovely assembly function

    push {r0-r2}    // Note lr is lost, but that's fine since this function never returns

    // Set the fault type in watchdog scratch 0
    ldr r2, =PANIC
    ldr r1, =WATCHDOG_BASE
    str r2, [r1, #WATCHDOG_SCRATCH0_OFFSET]

    // Save the panic message
    str r0, [r1, #WATCHDOG_SCRATCH1_OFFSET]

    // Save the program counter of the caller for the panic
    ldr r0, [sp, #0xC]  // Retreive lr from the stack of the calling function (since we're wrapped by panic in runtime.c)
    subs r0, r0, #2  // Since lr is for the next instruction, subtract 2 so we'll point to the previous
                     // Note this could be in the middle of the instruction if it's a 4 byte, but it'll at least let gdb go to the right place in code
    str r0, [r1, #WATCHDOG_SCRATCH2_OFFSET]

    // Load r12 with the real panic address
    // Can't use bl so we don't nuke the stack trace
    // But b won't relocate long enough, so we need to do register addressing
    ldr r0, =safety_panic_internal
    mov r12, r0

    pop {r0-r2}

    // Call the true panic
    bx r12


// Custom HardFault Exception Handler
// Logs details of the fault to scratch registers for easier debugging
// See safety.c Watchdog Crash Reporting Functions for more details on format

.global safety_hard_fault_handler
.type safety_hard_fault_handler,%function
.thumb_func
safety_hard_fault_handler:
    // Set the fault type in watchdog scratch 0
    ldr r0, =HARD_FAULT
    ldr r1, =WATCHDOG_BASE
    str r0, [r1, #WATCHDOG_SCRATCH0_OFFSET]

    // Save the program counter of the hard fault
    // Save #0xFFFFFFFF into the register in case msp is corrupted
    movs r0, #0
    subs r0, r0, #1
    str r0, [r1, #WATCHDOG_SCRATCH1_OFFSET]
    // Try to load program counter
    ldr r0, [sp, #0x18]
    subs r0, r0, #2  // Since lr is for the next instruction, subtract 2 so we'll point to the previous
                     // Note this could be in the middle of the instruction if it's a 4 byte, but it'll at least let gdb go to the right place in code
    str r0, [r1, #WATCHDOG_SCRATCH1_OFFSET]

1:
    bkpt #0
    b 1b

// Safety hard assert wrapper
// This catches the hard assert, instead of when it goes to panic, so the watchdog will contain the true caller's address
// Without this, the panic data would only say that a hard assert was called, and not who actually called it

.global __wrap_hard_assertion_failure
.type __wrap_hard_assertion_failure,%function
.thumb_func
__wrap_hard_assertion_failure:
    // Set the fault type in watchdog scratch 0
    ldr r0, =HARD_ASSERT
    ldr r1, =WATCHDOG_BASE
    str r0, [r1, #WATCHDOG_SCRATCH0_OFFSET]

    // Save the caller address
    mov r0, lr
    subs r0, r0, #2  // Since lr is for the next instruction, subtract 2 so we'll point to the previous
                     // Note this could be in the middle of the instruction if it's a 4 byte, but it'll at least let gdb go to the right place in code
    str r0, [r1, #WATCHDOG_SCRATCH1_OFFSET]

    ldr r0, =hard_assert_msg
    ldr r1, =safety_panic_internal
    bx r1

.data
hard_assert_msg:
.asciz "Hard assert"
