/*
 * Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "pico.h"
#include "pico/bootrom.h"
#include "hardware/regs/m0plus.h"
#include "hardware/regs/addressmap.h"
#include "hardware/regs/sio.h"
#include "hardware/regs/xip.h"
#include "pico/binary_info/defs.h"

#define BOOTROM_ROM_LOOKUP_PTR_ADDR 0x00000018
#define BOOTROM_ROM_FUNCTION_TABLE_ADDR 0x00000014

.syntax unified
.cpu cortex-m0plus
.thumb

.section .vectors, "ax"
.align 2

.global __vectors, __VECTOR_TABLE
__VECTOR_TABLE:
__vectors:
.word __StackTop
.word _reset_handler
.word isr_nmi
.word isr_hardfault
.word isr_invalid // Reserved, should never fire
.word isr_invalid // Reserved, should never fire
.word isr_invalid // Reserved, should never fire
.word isr_invalid // Reserved, should never fire
.word isr_invalid // Reserved, should never fire
.word isr_invalid // Reserved, should never fire
.word isr_invalid // Reserved, should never fire
.word isr_svcall
.word isr_invalid // Reserved, should never fire
.word isr_invalid // Reserved, should never fire
.word isr_pendsv
.word isr_systick
.word isr_irq0
.word isr_irq1
.word isr_irq2
.word isr_irq3
.word isr_irq4
.word isr_irq5
.word isr_irq6
.word isr_irq7
.word isr_irq8
.word isr_irq9
.word isr_irq10
.word isr_irq11
.word isr_irq12
.word isr_irq13
.word isr_irq14
.word isr_irq15
.word isr_irq16
.word isr_irq17
.word isr_irq18
.word isr_irq19
.word isr_irq20
.word isr_irq21
.word isr_irq22
.word isr_irq23
.word isr_irq24
.word isr_irq25
.word isr_irq26
.word isr_irq27
.word isr_irq28
.word isr_irq29
.word isr_irq30
.word isr_irq31

// all default exception handlers do nothing, and we can check for them being set to our
// default values by seeing if they point to somewhere between __defaults_isrs_start and __default_isrs_end
.global __default_isrs_start
__default_isrs_start:

// Declare a weak symbol for each ISR.
// By default, they will fall through to the undefined IRQ handler below (breakpoint),
// but can be overridden by C functions with correct name.

.macro decl_isr_bkpt name
.weak \name
.type \name,%function
.thumb_func
\name:
    bkpt #0
.endm

// these are separated out for clarity
decl_isr_bkpt isr_invalid
decl_isr_bkpt isr_nmi
decl_isr_bkpt isr_hardfault
decl_isr_bkpt isr_svcall
decl_isr_bkpt isr_pendsv
decl_isr_bkpt isr_systick

.global __default_isrs_end
__default_isrs_end:

.macro decl_isr name
.weak \name
.type \name,%function
.thumb_func
\name:
.endm

decl_isr isr_irq0
decl_isr isr_irq1
decl_isr isr_irq2
decl_isr isr_irq3
decl_isr isr_irq4
decl_isr isr_irq5
decl_isr isr_irq6
decl_isr isr_irq7
decl_isr isr_irq8
decl_isr isr_irq9
decl_isr isr_irq10
decl_isr isr_irq11
decl_isr isr_irq12
decl_isr isr_irq13
decl_isr isr_irq14
decl_isr isr_irq15
decl_isr isr_irq16
decl_isr isr_irq17
decl_isr isr_irq18
decl_isr isr_irq19
decl_isr isr_irq20
decl_isr isr_irq21
decl_isr isr_irq22
decl_isr isr_irq23
decl_isr isr_irq24
decl_isr isr_irq25
decl_isr isr_irq26
decl_isr isr_irq27
decl_isr isr_irq28
decl_isr isr_irq29
decl_isr isr_irq30
decl_isr isr_irq31

// All unhandled USER IRQs fall through to here
.global __unhandled_user_irq
.thumb_func
__unhandled_user_irq:
    bl __get_current_exception
    subs r0, #16
.global unhandled_user_irq_num_in_r0
unhandled_user_irq_num_in_r0:
    bkpt #0

// ----------------------------------------------------------------------------

.section .binary_info_header, "a"

// Header must be in first 256 bytes of main image (i.e. excluding flash boot2).
// For flash builds we put it immediately after vector table; for NO_FLASH the
// vectors are at a +0x100 offset because the bootrom enters RAM images directly
// at their lowest address, so we put the header in the VTOR alignment hole.

#if !PICO_NO_BINARY_INFO
binary_info_header:
.word BINARY_INFO_MARKER_START
.word __binary_info_start
.word __binary_info_end
.word data_cpy_table // we may need to decode pointers that are in RAM at runtime.
.word BINARY_INFO_MARKER_END
#endif

// ----------------------------------------------------------------------------

.section .entry_point, "ax"

// On flash builds, the vector table comes first in the image (conventional).
// On NO_FLASH builds, the reset handler section comes first, as the entry
// point is at offset 0 (fixed due to bootrom), and VTOR is highly-aligned.
// Image is entered in various ways:
//
// - NO_FLASH builds are entered from beginning by UF2 bootloader
//
// - Flash builds vector through the table into _reset_handler from boot2
//
// - Either type can be entered via _entry_point by the debugger, and flash builds
//   must then be sent back round the boot sequence to properly initialise flash

// ELF entry point:
.type _entry_point,%function
.thumb_func
.global _entry_point
_entry_point:
    // (Removed the PICO_NO_FLASH part, as we are a bootloader, there should be flash)

    // Debugger tried to run code after loading, so SSI is in 03h-only mode.
    // Go back through bootrom + boot2 to properly initialise flash.
    movs r0, #0
    ldr r1, =(PPB_BASE + M0PLUS_VTOR_OFFSET)
    str r0, [r1]
    ldmia r0!, {r1, r2}
    msr msp, r1
    bx r2


// Flash Reset Handler
// This copies the program into XIP RAM, fixes up VTOR, and begins exiting directly from XIP
.type _reset_handler,%function
.thumb_func
_reset_handler:
    // So we're in a very unstable state right now
    // VTOR is pointing to the table in flash, which has this reset handler
    // However all the other functions in VTOR, including hardfault, is referencing RAM, which hasn't been copied out yet
    // Copy the data out, fixup VTOR, then reset

    // Prepare for bootrom function lookup (needed for both hold bootrom and memcpy function)
    movs r0, #BOOTROM_ROM_FUNCTION_TABLE_ADDR
    ldrh r0, [r0]
    movs r3, #BOOTROM_ROM_LOOKUP_PTR_ADDR
    ldrh r3, [r3]

    // Only core 0 should run the C runtime startup code; core 1 is normally
    // sleeping in the bootrom at this point but check to be sure
    ldr r1, =(SIO_BASE + SIO_CPUID_OFFSET)
    ldr r1, [r1]
    cmp r1, #0
    bne hold_non_core0_in_bootrom

    // Lookup memcpy to copy out data
    ldr r1, =ROM_FUNC_MEMCPY44
    blx r3
    mov r4, r0

    // Disable XIP Cache so we can copy out bootloader to memory
    ldr r3, =XIP_CTRL_BASE
    // Stall if a cache flush is running to prevent memory corruption
    ldr r1, [r3, #XIP_FLUSH_OFFSET]
    // Now disable XIP cache
    movs r1, #1
    ldr r3, =(XIP_CTRL_BASE | REG_ALIAS_CLR_BITS)
    str r1, [r3, #XIP_CTRL_OFFSET]

    // Copy out the data_cpy_table, copying the data region and bootloader trampoline
    adr r5, data_cpy_table
1:
    // Grab the source address
    ldr r1, [r5, #0]

    // If source is NULL, then we've hit the end of the list
    cmp r1, #0
    beq 1f

    ldr r0, [r5, #4]  // Grab the destination
    ldr r2, [r5, #8]  // Grab the end of the destination
    subs r2, r2, r0   // Compute length of copyout
    blx r4            // Call memcpy

    // Repeat for the next entry
    adds r5, r5, #12  // Go to next entry
    b 1b

1:
    // Now that we've copied out the data, fixup the vector table pointed to by r3 and to be all in flash
    ldr r0, =__logical_binary_start
    ldr r1, =_ram_reset_entry
    str r1, [r0, #4]

    // Set VTOR and run the RAM reset handler
    ldr r1, =(PPB_BASE + M0PLUS_VTOR_OFFSET)
    str r0, [r1]
    ldmia r0, {r0, r1}
    msr msp, r0
    bx r1

// ----------------------------------------------------------------------------
// If core 1 somehow gets into crt0 due to a spectacular VTOR mishap, we need to
// catch it and send back to the sleep-and-launch code in the bootrom. Shouldn't
// happen (it should sleep in the ROM until given an entry point via the
// cross-core FIFOs) but it's good to be defensive.

hold_non_core0_in_bootrom:
    ldr r1, = 'W' | ('V' << 8)
    // Lookup bootrom (based on instruction flow before this call here)
    blx r3
    bx r0

// Data cpy table must live in flash so that the address isn't in RAM (since this is needed to get those maps)
.align 2
data_cpy_table:
.word __ram_copyout_source__
.word __ram_copyout_start__
.word __ram_copyout_end__

.word __boot_trampoline_source__
.word __boot_trampoline_entry__
.word __boot_trampoline_end__

.word 0 // null terminator

// ----------------------------------------------------------------------------

.section .boot_trampoline, "ax"

.extern app_boot2_copyout

.global _boot_trampoline
.type _boot_trampoline,%function
.thumb_func
_boot_trampoline:
    // Call bootrom flush cache
    // This reenables XIP Cache for the main application (this is why we need the trampoline)
    // Also clears the tags so it won't try to use the junk we copied in as valid cache
    movs r0, #BOOTROM_ROM_FUNCTION_TABLE_ADDR
    ldrh r0, [r0]
    ldr r1, =ROM_FUNC_FLASH_FLUSH_CACHE
    movs r3, #BOOTROM_ROM_LOOKUP_PTR_ADDR
    ldrh r3, [r3]
    blx r3
    blx r0

    // Call application boot2 (copied into boot2_copyout before calling the function)
    ldr r0, =(app_boot2_copyout + 1)
    blx r0

    // Jump into actual firmware
    movs r0, #0
    mov lr, r0
    ldr r0, =(__flash_app + 0x100)
    ldr r1, =(PPB_BASE + M0PLUS_VTOR_OFFSET)
    str r0, [r1]
    ldmia r0, {r0, r1}
    msr msp, r0
    bx r1


// ----------------------------------------------------------------------------

.section .reset, "ax"

// Reset handler:
// - disables XIP
// - clears .bss
// - calls runtime_init
// - calls main
// - calls exit (which should eventually hang the processor via _exit)

.type _ram_reset_entry,%function
.thumb_func
_ram_reset_entry:
    // Begin actual environment setup now that we're running in RAM with all our code
    // No need to copy .data, it was copied in with the code

    // First disable XIP now that we don't need to execute from flash anymore
    movs r0, #BOOTROM_ROM_FUNCTION_TABLE_ADDR
    ldrh r0, [r0]
    ldr r1, =ROM_FUNC_FLASH_EXIT_XIP
    movs r3, #BOOTROM_ROM_LOOKUP_PTR_ADDR
    ldrh r3, [r3]
    blx r3
    blx r0

    // Zero out the BSS
    ldr r1, =__bss_start__
    ldr r2, =__bss_end__
    movs r0, #0
    b bss_fill_test
bss_fill_loop:
    stm r1!, {r0}
bss_fill_test:
    cmp r1, r2
    bne bss_fill_loop

platform_entry: // symbol for stack traces
    // Use 32-bit jumps, in case these symbols are moved out of branch range
    // (e.g. if main is in SRAM and crt0 in flash)
    ldr r1, =runtime_init
    blx r1
    ldr r1, =main
    blx r1
    ldr r1, =_exit
    blx r1
    // exit should not return.  If it does, hang the core.
    // (fall thru into our hang _exit impl
.weak _exit
.type _exit,%function
.thumb_func
_exit:
1: // separate label because _exit can be moved out of branch range
    bkpt #0
    b 1b

// ----------------------------------------------------------------------------
// Provide safe defaults for _exit and runtime_init
// Full implementations usually provided by platform.c

.weak runtime_init
.type runtime_init,%function
.thumb_func
runtime_init:
    bx lr

.global __get_current_exception
.thumb_func
__get_current_exception:
    mrs  r0, ipsr
    uxtb r0, r0
    bx   lr


// ----------------------------------------------------------------------------
// Stack/heap dummies to set size

.section .stack
// align to allow for memory protection (although this alignment is pretty much ignored by linker script)
.align 5
    .equ StackSize, PICO_STACK_SIZE
.space StackSize

.section .heap
.align 2
    .equ HeapSize, PICO_HEAP_SIZE
.space HeapSize
