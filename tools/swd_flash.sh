#!/bin/sh
if [ -z "$1" ]; then
    echo "Invalid Syntax: $0 [elf file]"
    exit 1
fi
set -e
firmware_img=$(realpath "$1")

cd ~/Downloads/openocd/tcl
echo "Resetting..."
../src/openocd -f interface/picoprobe.cfg -f target/rp2040-rescue.cfg -c 'init' -c 'exit'

echo "Programming..."
../src/openocd -f interface/picoprobe.cfg -f target/rp2040.cfg -c "adapter speed 5000" -c 'init' -c "halt" -c "flash probe 0" -c "flash write_image $firmware_img"
echo "Verifying..."
../src/openocd -f interface/picoprobe.cfg -f target/rp2040.cfg -c "adapter speed 5000" -c "init" -c "halt" -c "flash probe 0" -c "flash verify_image $firmware_img" -c 'reset' -c 'exit'
