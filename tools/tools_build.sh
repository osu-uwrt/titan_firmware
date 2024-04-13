#!/bin/bash

if [ -z "$BASH" ]; then
    echo "This script must be ran in bash"
    exit 1
fi

if [ -z "$1" ]; then
    echo "Invalid syntax!"
    echo "Usage: $0 [install dir]"
    echo
    echo "Builds the titan_firmware tools and places them in [install dir]"
    echo "This is useful to deploy on the robot computer to allow uploading OTA images and debugging the systems while running"
    exit 1
fi

set -e

INSTALL_DIR="$(realpath "$1")"
WORK_DIR="$(mktemp -d -t titan_fw_build_XXXXXXXXXX)"

build_tool() {
    mkdir build/
    pushd build/
    cmake .. -DCMAKE_INSTALL_PREFIX="$INSTALL_DIR"
    cmake --build . --config Release -- -j $(nproc)
    cmake --install .
    popd
}

mkdir -p "$INSTALL_DIR"

# Initialize Directory
git clone --depth=1 --branch=master https://github.com/osu-uwrt/titan_firmware "$WORK_DIR/titan_firmware"
pushd "$WORK_DIR/titan_firmware"

# Setup submodules
git submodule update --init --depth=1
pushd "lib/pico-sdk"
git submodule update --init --depth=1
popd

# Build upload tool
pushd tools/upload_tool
build_tool
popd

# Build canmore_cli
pushd tools/canmore_cli
build_tool
popd

# Cleanup
popd
rm -rf "$WORK_DIR"
