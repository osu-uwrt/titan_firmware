#include "hardware/regs/addressmap.h"
#include "hardware/regs/watchdog.h"

#define IN_ROS_TRANSPORT_LOOP   0x1035006

.data
.type prev_scratch_reg,%object
.align 4
.size prev_scratch_reg, 4
prev_scratch_reg:
.word 0

.type prev_link_reg,%object
.align 4
.size prev_scratch_reg, 4
prev_link_reg:
.word 0


.text

.macro hook_blocking_ros_transport_func name:req
.extern __real_\name
.global __wrap_\name
.type __wrap_\name,%function
.thumb_func
__wrap_\name:

    push {r0-r3}

    // First check that this doesn't get recursively called
    ldr r1, =(prev_link_reg)
    ldr r0, [r1]
    cmp r0, #0
    beq 0f
    bkpt #0
0:
    // Then save link register
    mov r0, lr
    str r0, [r1]
    
    // Save scratch0 to prev value
    ldr r1, =(WATCHDOG_BASE + WATCHDOG_SCRATCH0_OFFSET)
    ldr r0, [r1]
    ldr r2, =(prev_scratch_reg)
    str r0, [r2]

    // Set value for when in loop
    ldr r0, =(IN_ROS_TRANSPORT_LOOP)
    str r0, [r1]

    pop {r0-r3}

    // Call real function with all proper values
    bl __real_\name

    push {r0-r3}
    // Restore scratch0
    ldr r1, =(WATCHDOG_BASE + WATCHDOG_SCRATCH0_OFFSET)
    ldr r2, =(prev_scratch_reg)
    ldr r0, [r2]
    str r0, [r1]

    // Pull link register and clear prev value for checks
    ldr r1, =(prev_link_reg)
    ldr r2, [r1]
    mov r12, r2
    mov r0, #0
    str r0, [r1]
    pop {r0-r3}

    bx r12
.endm

hook_blocking_ros_transport_func uxr_run_session_until_confirm_delivery
hook_blocking_ros_transport_func uxr_ping_agent_session

#define HARD_FAULT_DATA 0x1035004

// Custom HardFault Exception Handler
// Logs details of the fault to scratch registers for easier debugging
// See safety.c Watchdog Crash Reporting Functions for more details on format

.global safety_hard_fault_handler
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
