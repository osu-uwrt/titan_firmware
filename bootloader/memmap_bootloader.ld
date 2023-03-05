/* Based on GCC ARM embedded samples.
   Defines the following symbols for use by code:
    __ram_copyout_start__
    __ram_copyout_source__
    __ram_copyout_end__
    __ram_text_start__
    __ram_text_end__
    __etext
    __data_start__
    __data_end__
    __bss_start__
    __bss_end__
    __end__
    end
    __StackLimit
    __StackTop
    __stack (== StackTop)
*/

MEMORY
{
    /* Flash isn't executable as it needs to be able to be overwritten by the bootloader */
    FLASH_BL(r)    : ORIGIN = 0x10000000, LENGTH = 16k
    FLASH_APP(r)   : ORIGIN = 0x10004000, LENGTH = 2032k
    RAM(rwx)       : ORIGIN = 0x20000000, LENGTH = 256k
    XIP_RAM(rwx)   : ORIGIN = 0x15000000, LENGTH = 16k
    USB_RAM(rwx)   : ORIGIN = 0x50100000, LENGTH = 4k
    SCRATCH_X(rwx) : ORIGIN = 0x20040000, LENGTH = 4k
    SCRATCH_Y(rwx) : ORIGIN = 0x20041000, LENGTH = 4k
}

ENTRY(_entry_point)

SECTIONS
{
    /* Second stage bootloader is prepended to the image. It must be 256 bytes big
       and checksummed. It is usually built by the boot_stage2 target
       in the Raspberry Pi Pico SDK
    */

    .flash_begin : {
        __flash_binary_start = .;
    } > FLASH_BL

    .boot2 : {
        __boot2_start__ = .;
        KEEP (*(.boot2))
        __boot2_end__ = .;
    } > FLASH_BL

    ASSERT(__boot2_end__ - __boot2_start__ == 256,
        "ERROR: Pico second stage bootloader must be 256 bytes in size")

    /* The second stage will always enter the image at the start of .text.
       The debugger will use the ELF entry point, which is the _entry_point
       symbol if present, otherwise defaults to start of .text.
       This can be used to transfer control back to the bootrom on debugger
       launches only, to perform proper flash setup.
    */

    .text : {
        . = ALIGN(4);
        __ram_copyout_start__ = .;
        __ram_text_start__ = .;
        __logical_binary_start = .;
        KEEP (*(.vectors))
        KEEP (*(.reset))
        *(.text*)
        . = ALIGN(4);
        __ram_text_end__ = .;
    } > XIP_RAM AT> FLASH_BL
    __ram_copyout_source__ = LOADADDR(.text);

    .data : {
        __data_start__ = .;
        *(.time_critical*)

        . = ALIGN(4);
        *(.rodata*)
        . = ALIGN(4);
        *(.data*)

        . = ALIGN(4);
        /* All data end */
        __data_end__ = .;
        __ram_copyout_end__ = .;
    } > XIP_RAM AT> FLASH_BL

    /* __etext is the name of the .data init source pointer (...) */
    __etext = LOADADDR(.data);

    .rodata : {
        /* Put the entry point in flash, required so the debugger won't try to jump into RAM after programming.
        Also tells elf2uf2 that this is a flash, not a RAM program */
        KEEP (*(.entry_point))
        *(.init)
        *(.fini)
        . = ALIGN(4);

        /* segments not marked as .flashdata are instead pulled into .data (in RAM) to avoid accidental flash accesses */
        *(SORT_BY_ALIGNMENT(SORT_BY_NAME(.flashdata*)))
        . = ALIGN(4);
    } > FLASH_BL

    .uninitialized_data (COPY): {
        . = ALIGN(4);
        *(.uninitialized_data*)
    } > USB_RAM

    .bss  : {
        . = ALIGN(4);
        __bss_start__ = .;
        *(SORT_BY_ALIGNMENT(SORT_BY_NAME(.bss*)))
        *(COMMON)
        . = ALIGN(4);
        __bss_end__ = .;
    } > USB_RAM

    /* .stack*_dummy section doesn't contains any symbols. It is only
     * used for linker to calculate size of stack sections, and assign
     * values to stack symbols later
     *
     * stack1 section may be empty/missing if platform_launch_core1 is not used */
    .stack_dummy (COPY):
    {
        *(.stack*)
    } > SCRATCH_Y

    /* End of RAM Calculations */
    .usbram_end (COPY):
    {
        __end__ = .;
        end = __end__;
    } > USB_RAM

    .flash_end : {
        __flash_binary_end = .;
    } > FLASH_BL

    .xip_end : {
        __ram_copyout_end = .;
    } > XIP_RAM

    /* stack limit is poorly named, but historically is maximum heap ptr */
    __StackLimit = ORIGIN(RAM) + LENGTH(RAM);
    __StackTop = ORIGIN(SCRATCH_Y) + LENGTH(SCRATCH_Y);
    __StackBottom = __StackTop - SIZEOF(.stack_dummy);
    PROVIDE(__stack = __StackTop);

    /* Ensure data and RAM is continuous */
    /* TODO: Fix to be == rather than <= */
    ASSERT(__ram_text_end__ <= __data_start__, "Text and data are not continuous")
    /* Check if fits in USB RAM limit */
    ASSERT(ORIGIN(USB_RAM) + LENGTH(USB_RAM) >= __end__, "region USB RAM overflowed")
    /* Check if fits in XIP RAM limit */
    ASSERT(ORIGIN(XIP_RAM) + LENGTH(XIP_RAM) >= __ram_copyout_end, "region XIP RAM overflowed")
    /* Check if fits in bootloader section of flash */
    ASSERT(ORIGIN(FLASH_BL) + LENGTH(FLASH_BL) >= __flash_binary_end, "Bootloader region of FLASH overflowed")

}

