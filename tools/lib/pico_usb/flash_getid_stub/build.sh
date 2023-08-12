#!/bin/sh
set -e
arm-none-eabi-gcc -I../../../../lib/pico-sdk/src/rp2040/hardware_regs/include -nostartfiles -nodefaultlibs -nostdlib -ffreestanding -fPIC -mthumb -c flash_getid.S -o flash_getid.o
arm-none-eabi-objcopy -O binary -j .text flash_getid.o flash_getid.bin
xxd -i flash_getid.bin > temp
echo "// Compiled code output of flash_getid.S
// Reads the unique flash id and flash jedec id of an RP2040 in bootsel mode
" | cat - temp > flash_getid_compiled.c
rm flash_getid.o flash_getid.bin temp
