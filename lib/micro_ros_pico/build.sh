#!/bin/bash

set -e

cd "$(dirname "$0")"

LIBRARY_GEN_SCRIPT="microros_static_library/library_generation/library_generation.sh"
if ! [ -f "$LIBRARY_GEN_SCRIPT" ]; then
    echo "Could not find $LIBRARY_GEN_SCRIPT"
    echo "Ensure build.sh is in the correct directory"
    exit 1
fi

if ! [ -x "$LIBRARY_GEN_SCRIPT" ]; then
    echo "$LIBRARY_GEN_SCRIPT is not executable!"
    echo "Chmod the file and try again"
    exit 1
fi

docker pull microros/micro_ros_static_library_builder:humble
docker run -it --rm -v $(pwd):/project microros/micro_ros_static_library_builder:humble
