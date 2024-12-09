#!/bin/sh
if [ -z "$1" ]; then
    echo "Invalid Syntax: $0 [elf file]"
    exit 1
fi
gdb-multiarch -ex "target extended-remote :3333" "$1"
