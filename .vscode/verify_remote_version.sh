#!/bin/bash

# Verifies that the firmware version on the GDB remote target matches the locally compiled version

REMOTE_URL="localhost:3333"
LOCAL_FILE="$1"
TIMEOUT_SEC=5

if [ -z "$1" ]; then
    echo "Expected filename!"
    echo "Usage: $0 [elf file]"
    exit 2
fi

LOCAL_VERSION="$(gdb-multiarch -batch -silent -ex 'printf "%s", FULL_BUILD_TAG' -ex quit "$LOCAL_FILE")"
REMOTE_VERSION="$(gdb-multiarch -batch-silent -ex "set tcp connect-timeout $TIMEOUT_SEC" -ex "target remote $REMOTE_URL" -ex "set logging file /dev/stdout" -ex "set logging enabled on" -ex 'printf "%s", FULL_BUILD_TAG' -ex "set logging enabled off" -ex quit "$LOCAL_FILE")"

if [[ "$LOCAL_VERSION" != "$REMOTE_VERSION" ]]; then
    echo -e "\033[91mVersion on remote does not match local version!\033[0m" >&2
    echo -e "\033[32mLocal Version:\033[0m  $LOCAL_VERSION"
    echo -e "\033[32mRemote Version:\033[0m $REMOTE_VERSION"
    exit 1
fi
