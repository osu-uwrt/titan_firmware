#include "hardware/regs/addressmap.h"
#include "hardware/regs/ssi.h"
#include "hardware/regs/io_qspi.h"

.cpu cortex-m0plus
.text
.thumb

#define BOOTROM_ROM_LOOKUP_PTR_ADDR     0x00000018
#define BOOTROM_ROM_FUNCTION_TABLE_ADDR 0x00000014
#define ROM_FUNC_CONNECT_INTERNAL_FLASH ROM_TABLE_CODE('I', 'F')
#define ROM_FUNC_FLASH_EXIT_XIP         ROM_TABLE_CODE('E', 'X')

#define ROM_TABLE_CODE(c1, c2) ((c1) | ((c2) << 8))

// ====================
// Main Function
// ====================

.type entry,%function
.global entry
.thumb_func
entry:
    push {r6, r7, lr}

    // Prepare for bootrom function lookup to call connect internal flash and exit xip
    mov    r6, #BOOTROM_ROM_FUNCTION_TABLE_ADDR
    ldrh   r6, [r6]
    mov    r7, #BOOTROM_ROM_LOOKUP_PTR_ADDR
    ldrh   r7, [r7, #0]

    // Call connect internal flash
    mov    r0, r6
    ldr    r1, =ROM_FUNC_CONNECT_INTERNAL_FLASH
    blx    r7  // Call lookup
    blx    r0  // Call resulting function

    // Call Flash Exit XIP
    mov    r0, r6
    ldr    r1, =ROM_FUNC_FLASH_EXIT_XIP
    blx    r7
    blx    r0

    // === Lookup Flash ID ===
    // Drive CS low
    mov    r0, #0
    bl     set_cs

    // Send read flash unique ID command
    mov    r0, #0x4B
    bl     flash_xfer

    // Transfer 4 dummy bytes
    mov    r7, #4
1:
    mov    r0, #0
    bl     flash_xfer
    sub    r7, r7, #1
    bne    1b

    // Read out the 64-bit unique ID into buffer immediately after program
    adr    r6, output_buffer
    mov    r7, #8
1:
    mov    r0, #0
    bl     flash_xfer
    strb   r0, [r6]
    add    r6, #1
    sub    r7, #1
    bne    1b

    // Drive CS back high
    mov    r0, #1
    bl     set_cs

    // Delay for a bit before sending next command
    mov    r7, #0x80
1:
    sub    r7, #1
    bne    1b

    // === Lookup JEDEC ID ===
    // Drive CS Low
    mov    r0, #0
    bl     set_cs

    // Send Lookup JEDEC ID Command
    mov    r0, #0x9f
    bl     flash_xfer

    // Read the 3 byte JEDEC ID out from the chip right
    // Note writing immediately after flash ID in the output buffer
    mov    r7, #3
1:
    mov    r0, #0
    bl     flash_xfer
    strb   r0, [r6]
    add    r6, #1
    sub    r7, #1
    bne    1b

    // Set last byte to 0 to make our lives a lot easier
    mov    r0, #0
    strb   r0, [r6]

    // Drive CS back high
    mov    r0, #1
    bl     set_cs

    pop    {r6, r7, pc}

// ====================
// Utility Functions
// ====================

set_cs:
    // Set force high/low depending on input argument
    cmp    r0, #0
    beq    1f
    mov    r0, #IO_QSPI_GPIO_QSPI_SS_CTRL_OUTOVER_VALUE_HIGH
    b      2f
1:
    mov    r0, #IO_QSPI_GPIO_QSPI_SS_CTRL_OUTOVER_VALUE_LOW
2:
    lsl    r0, r0, #IO_QSPI_GPIO_QSPI_SS_CTRL_OUTOVER_LSB

    // Load the destination register address
    ldr    r1, =(IO_QSPI_BASE + IO_QSPI_GPIO_QSPI_SS_CTRL_OFFSET)

    // Perform read/modify/write on register
    ldr    r2, =(~IO_QSPI_GPIO_QSPI_SS_CTRL_OUTOVER_BITS)
    ldr    r3, [r1]
    and    r3, r2
    orr    r3, r0
    str    r3, [r1]

    mov    pc, lr

flash_xfer:
    ldr    r1, =XIP_SSI_BASE

    // Wait for TX fifo to be ready
    mov    r3, #SSI_SR_TFNF_BITS
1:
    ldr    r2, [r1, #SSI_SR_OFFSET]
    and    r2, r3
    beq    1b

    // Write data to transfer
    str    r0, [r1, #SSI_DR0_OFFSET]

    // Wait for RX fifo ready flag
    mov    r3, #SSI_SR_RFNE_BITS
1:
    ldr    r2, [r1, #SSI_SR_OFFSET]
    and    r2, r3
    beq    1b

    // Read out data from transfer
    ldr    r0, [r1, #SSI_DR0_OFFSET]

    mov    pc, lr

literals:
.ltorg

output_buffer:
