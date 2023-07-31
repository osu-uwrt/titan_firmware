#!/bin/sh
set -e
arm-none-eabi-gcc -I../../lib/pico-sdk/src/rp2040/hardware_regs/include -nostartfiles -nodefaultlibs -nostdlib -ffreestanding -fPIC -mthumb -c flash.c -o flash.o
arm-none-eabi-ld -o flash.elf -T linker.ld flash.o
arm-none-eabi-objcopy -O binary -j .text flash.elf flash.bin
xxd -i flash.bin > temp
echo "// Compiled code output of bootloader-serial-read/flash.c
// Reads the flash id of an RP2040 in bootsel mode
" | cat - temp > ../read_flash.h
rm flash.elf flash.o flash.bin temp