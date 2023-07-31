MEMORY {
    ROM(rx) : ORIGIN = 0x00000000, LENGTH = 16K
    SRAM(rwx) : ORIGIN = 0x20000000, LENGTH = 264K
    USBRAM(rw) : ORIGIN = 0x50100400, LENGTH = 3K
}

SECTIONS {
    /* This determines the offset from the SRAM base the serial will be written to */
    _serial_write_offset = 0x1000;

    . = ORIGIN(SRAM);
    .text : {
        KEEP(*(.text.entry))
        *(.text)
    } >SRAM

    .data : {
        *(.data*)
    } >USBRAM

    serial_write_address = ORIGIN(SRAM) + _serial_write_offset;
    ASSERT(SIZEOF(.text) < _serial_write_offset, "ERROR: Text is too large (overflows into serial output)")
    ASSERT(SIZEOF(.data) == 0,
        "ERROR: do not use static memory in bootrom! (.data)")
}
