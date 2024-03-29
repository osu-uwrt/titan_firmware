#!/bin/bash

if [ -z "$2" ]; then
    echo "Usage: $0 [elf file] [hex addr]"
    exit 1
fi

gdb -q "$1" -ex "printf \"Full Build Tag: %s\n\n\", FULL_BUILD_TAG" -ex "x /s $2" -ex "quit"
