#include "safety_helper.h"

#include "hardware/regs/addressmap.h"
#include "hardware/regs/sio.h"
#include "hardware/regs/syscfg.h"
#include "hardware/regs/timer.h"
#include "hardware/regs/intctrl.h"
#include "hardware/regs/watchdog.h"
#include "pico/asm_helper.S"
#include "titan/safety_magic_values.h"

#define CRASH_LOGGED_ADDR (SIO_BASE + SIO_SPINLOCK0_OFFSET + (SAFETY_CRASH_LOGGED_SPINLOCK_ID * 4))

pico_default_asm_setup

.text

// Safety panic wrapper
// This extracts the calling address, as well as the panic message, and saves it to the watchdog
// Then calls the internal panic function which will print and halt the CPU accordingly

// DO NOT CALL THIS FUNCTION DIRECTLY! Use panic() instead
// This assumes that it is called via panic, and rolls back the stack a bit to extract lr

.global safety_panic
.type safety_panic,%function
.thumb_func
safety_panic:
    // Note this is done in assembly so we can pull the lr out of the code
    // Additionally, we don't want to pass this data to the C function so we don't mess up the varargs
    // Because of this, we get this lovely assembly function

    push {r0-r2,lr}

    // Only log the crash if one hasn't been logged before (the spin lock can be acquired)
    ldr r2, =CRASH_LOGGED_ADDR
    ldr r2, [r2]
    cmp r2, #0
    beq 1f

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

    // Now halt the other core
    // Gaurenteed to only clobber r0 & r1
    // We need to save lr though for proper stack trace recovery
    mov r2, lr
    bl safety_halt_other_core
    mov lr, r2

1:
    // Load r12 with the real panic function address
    // Can't use bl so we don't nuke the stack trace
    // But b won't relocate long enough, so we need to do register addressing
    ldr r0, =safety_panic_internal
    mov r12, r0

    pop {r0-r2}

    // Call the true panic
    bx r12

// Because of the way they designed the pico-sdk, the stack trace gets corrupted when you use a custom panic
// So, we're going to also use a custom __wrap_panic that won't corrupt the stack trace
// However, we still need to have the panic override in case the runtime.c panics, which will circumvent the linker
// override

.global __wrap_panic
.type __wrap_panic,%function
.thumb_func
__wrap_panic:
    // Note this is done in assembly so we can pull the lr out of the code
    // Additionally, we don't want to pass this data to the C function so we don't mess up the varargs
    // Because of this, we get this lovely assembly function

    push {r0-r2}

    // Only log the crash if one hasn't been logged before (the spin lock can be acquired)
    ldr r2, =CRASH_LOGGED_ADDR
    ldr r2, [r2]
    cmp r2, #0
    beq 1f

    // Set the fault type in watchdog scratch 0
    ldr r2, =PANIC
    ldr r1, =WATCHDOG_BASE
    str r2, [r1, #WATCHDOG_SCRATCH0_OFFSET]

    // Save the panic message
    str r0, [r1, #WATCHDOG_SCRATCH1_OFFSET]

    // Save the program counter of the caller for the panic
    mov r0, lr
    subs r0, r0, #2  // Since lr is for the next instruction, subtract 2 so we'll point to the previous
                     // Note this could be in the middle of the instruction if it's a 4 byte, but it'll at least let gdb go to the right place in code
    str r0, [r1, #WATCHDOG_SCRATCH2_OFFSET]

    // Now halt the other core
    // Gaurenteed to only clobber r0 & r1
    // We need to save lr though for proper stack trace recovery
    mov r2, lr
    bl safety_halt_other_core
    mov lr, r2

1:
    // Load r12 with the real panic function address
    // Can't use bl so we don't nuke the stack trace
    // But b won't relocate long enough, so we need to do register addressing
    // But we can use r12 (that's what its for in the arm ABI)
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
    // Only log the crash if one hasn't been logged before (the spin lock can be acquired)
    ldr r0, =CRASH_LOGGED_ADDR
    ldr r0, [r0]
    cmp r0, #0
    beq 1f

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

    // Now halt the other core
    // Gaurenteed to only clobber r0 & r1
    // We need to save lr though for proper stack trace recovery
    mov r2, lr
    bl safety_halt_other_core
    mov lr, r2

1:
    // ========================================
    // ========== YOUR CODE CRASHED! ==========
    // ========================================
    // If you've gotten here, then your code crashed the CPU
    // Check the stack trace to see what happened
    bkpt #0
    b 1b

// Safety assertion wrapper
// This catches any assertion failures and logs it to the watchdog, before forwarding it
// to the real assertion handler provided by the pico-sdk

.extern __real___assert_func

.global __wrap___assert_func
.type __wrap___assert_func,%function
.thumb_func
__wrap___assert_func:
    push {r0-r3}

    // Only log the crash if one hasn't been logged before (the spin lock can be acquired)
    ldr r2, =CRASH_LOGGED_ADDR
    ldr r2, [r2]
    cmp r2, #0
    beq 1f

    // Set the fault type in watchdog scratch 0
    ldr r2, =ASSERT_FAIL
    ldr r3, =WATCHDOG_BASE
    str r2, [r3, #WATCHDOG_SCRATCH0_OFFSET]

    // Save the asserted file
    str r0, [r3, #WATCHDOG_SCRATCH1_OFFSET]

    // Save the asserted line number
    str r1, [r3, #WATCHDOG_SCRATCH2_OFFSET]

    // Now halt the other core
    // Gaurenteed to only clobber r0 & r1
    // We need to save lr though for proper stack trace recovery
    mov r2, lr
    bl safety_halt_other_core
    mov lr, r2

1:
    // Load r12 with the real __assert_func provided by the pico sdk
    // Can't use bl so we don't nuke the stack trace
    // But b won't relocate long enough, so we need to do register addressing
    // But we can use r12 (that's what its for in the arm ABI)
    ldr r0, =__real___assert_func
    mov r12, r0

    pop {r0-r3}

    // Call the true __assert_func
    bx r12

// Safety hard assert wrapper
// This catches the hard assert, instead of when it goes to panic, so the watchdog will contain the true caller's address
// Without this, the panic data would only say that a hard assert was called, and not who actually called it
// which makes it kind of annoying to debug

.global __wrap_hard_assertion_failure
.type __wrap_hard_assertion_failure,%function
.thumb_func
__wrap_hard_assertion_failure:
    // Only log the crash if one hasn't been logged before (the spin lock can be acquired)
    ldr r0, =CRASH_LOGGED_ADDR
    ldr r0, [r0]
    cmp r0, #0
    beq 1f

    // Set the fault type in watchdog scratch 0
    ldr r0, =HARD_ASSERT
    ldr r1, =WATCHDOG_BASE
    str r0, [r1, #WATCHDOG_SCRATCH0_OFFSET]

    // Save the caller address
    mov r0, lr
    subs r0, r0, #2  // Since lr is for the next instruction, subtract 2 so we'll point to the previous
                     // Note this could be in the middle of the instruction if it's a 4 byte, but it'll at least let gdb go to the right place in code
    str r0, [r1, #WATCHDOG_SCRATCH1_OFFSET]

    // Now halt the other core
    // Gaurenteed to only clobber r0 & r1
    // We need to save lr though for proper stack trace recovery
    mov r2, lr
    bl safety_halt_other_core
    mov lr, r2

1:
    ldr r0, =hard_assert_msg
    ldr r1, =safety_panic_internal
    bx r1

.section .rodata
hard_assert_msg:
.asciz "Hard assert"

// Core 0 Watchdog Imminent NMI Handler
// This will fire 1us before the watchdog is set to go off
// This aids in debugging watchdog timeouts, as this records the current execution location right before reset
// This additionally lives in RAM, since we don't have time to wait for flash to run (we've got 1us to save the address)

// Note this shouldn't need to be deregistered for panics or hard faults, even though they time out the watchdog
// This is because, even though NMI can preempt even a hardfault, whenever a breakpoint fires (which occurs for all
// critical faults), it will hard fault. Because hard fault has a breakpoint, this will escilate into a core lockup,
// which immediately stops all execution, including NMIs.

.section RAM_SECTION_NAME(safety_nmi_handler), "ax"
.global safety_nmi_handler
.type safety_nmi_handler,%function
.thumb_func
safety_nmi_handler:
    // Only log the crash if one hasn't been logged before (the spin lock can be acquired)
    ldr r0, =CRASH_LOGGED_ADDR
    ldr r0, [r0]
    cmp r0, #0
    beq 1f

    // Set the fault type in watchdog scratch 0
    ldr r0, =WATCHDOG_TIMEOUT
    ldr r1, =WATCHDOG_BASE
    str r0, [r1, #WATCHDOG_SCRATCH0_OFFSET]

    // Store the interrupted instruction in scratch1
    ldr r0, [sp, #0x18]
    subs r0, r0, #2  // Since lr is for the next instruction, subtract 2 so we'll point to the previous
                     // Note this could be in the middle of the instruction if it's a 4 byte, but it'll at least let gdb go to the right place in code
    str r0, [r1, #WATCHDOG_SCRATCH1_OFFSET]

    // Now halt the other core
    // Gaurenteed to only clobber r0 & r1
    // We need to save lr though for proper stack trace recovery
    mov r2, lr
    bl safety_halt_other_core
    mov lr, r2
    b 2f

1:
    // Check if we need to capture the address we were called from core 1
    // This is set if safety notices that core 1 has timed out, and not checked in within the timeout
    // It will then panic, and bring down both cores
    // When core 1 is brought down, it'll execute this handler (and enter this section here)
    // That means its okay for us to overwrite the panic data in the watchdog with our last address before entering nmi
    ldr r0, =core1_nmi_capture_addr
    ldrb r0, [r0]
    cmp r0, #0
    beq 2f

    // Make sure we're core 1 (instead of core 0 entering the nmi right before watchdog timeout)
    ldr r0, =SIO_BASE
    ldr r0, [r0, #SIO_CPUID_OFFSET]
    cmp r0, #0
    beq 2f

    // Now we can save the location of the crash
    ldr r0, =CORE1_TIMEOUT
    ldr r1, =WATCHDOG_BASE
    str r0, [r1, #WATCHDOG_SCRATCH0_OFFSET]

    // Store the interrupted instruction in scratch1
    ldr r0, [sp, #0x18]
    subs r0, r0, #2  // Since lr is for the next instruction, subtract 2 so we'll point to the previous
                     // Note this could be in the middle of the instruction if it's a 4 byte, but it'll at least let gdb go to the right place in code
    str r0, [r1, #WATCHDOG_SCRATCH1_OFFSET]

2:
    // ========================================
    // ========== YOUR CODE CRASHED! ==========
    // ========================================
    // If you've gotten here, there's two things that might have happened:
    //  1. If you are the first core to crash, this means you timed out the watchdog (only on core 0)
    //  2. If the other core crashed first, then it sent an NMI to this core to halt it and wait for the watchdog
    //     - You can tell this is the case if the other core is not an NMI
    //     - If both cores are stopped here, then core 0 timed out the watchdog, and core 1 was then stopped right after
    bkpt #0
    b 1b

.bss
.global core1_nmi_capture_addr
core1_nmi_capture_addr:
.byte 0

// Function to send NMI event to the other core
// This should only be called AFTER the safety crash logged spinlock has been acquired
// This will cause the nmi handler to fire and then immediately breakpoint/halt since

// This function is gaurenteed to only clobber r0 & r1

.section RAM_SECTION_NAME(safety_halt_other_core), "ax"
.global safety_halt_other_core
.type safety_halt_other_core,%function
.thumb_func
safety_halt_other_core:
    ldr r0, =SIO_BASE
    ldr r0, [r0, #SIO_CPUID_OFFSET]
    cmp r0, #0
    bne 1f

    // We're running on core 0
    // Only fire the nmi if the other core is running
    // Prevents weird stuff where the bootrom NMI enables the clocks and clears syscfg
    // It's probably fine, but it's better to be safe
    // (technically can still happen though if core 0 crashes before core 1 sets the vtor, but that's so rare it's fine)
    ldr r0, =safety_multicore_running
    ldrb r0, [r0]
    cmp r0, #0
    beq 2f

    // So core 1 is running, now we need to stop it
    // We're going to use the NMI to do this
    ldr r0, =SYSCFG_BASE

    // Disable NMI on core 0
    movs r1, #0
    str r1, [r0, #SYSCFG_PROC0_NMI_MASK_OFFSET]

    // Enable NMI on core 1 for the timer
    movs r1, #1
    lsls r1, r1, #(TIMER_IRQ_0 + SAFETY_WATCHDOG_ALARM_NUM)
    str r1, [r0, #SYSCFG_PROC1_NMI_MASK_OFFSET]

1:
    // Now we're going to trigger the NMI
    // Note we jump here directly on core 1, since this will have the same effect, where it'll immediately halt the core
    // since core 0 is already subscribed to the NMI to monitor watchdog timeouts
    // This can be done by simply forcing the timer interrupt
    ldr r0, =TIMER_BASE
    movs r1, #(1 << SAFETY_WATCHDOG_ALARM_NUM)
    str r1, [r0, #TIMER_INTF_OFFSET]

2:
    bx lr

