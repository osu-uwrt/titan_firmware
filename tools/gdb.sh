#!/bin/sh
if [ -z "$1" ]; then
    echo "Invalid Syntax: $0 [elf file]"
    exit 1
fi
"${GDB:-$(command -v gdb-multiarch || echo arm-none-eabi-gdb)}" -ex "target extended-remote :3333" "$1"
