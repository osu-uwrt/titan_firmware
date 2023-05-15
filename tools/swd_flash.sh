#!/bin/sh
if [ -z "$1" ]; then
    echo "Invalid Syntax: $0 [elf file]"
    exit 1
fi
firmware_img=$(realpath "$1")

cd ~/Downloads/openocd/tcl
../src/openocd -f interface/cmsis-dap.cfg -f target/rp2040.cfg -c "adapter speed 5000" -c 'init' -c 'reset halt' -c "program $firmware_img" -c 'reset' -c 'exit'
