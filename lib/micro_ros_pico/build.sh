#!/bin/bash

cd "$(dirname "$0")"

sudo chmod +x ./microros_static_library/library_generation/library_generation.sh

docker pull microros/micro_ros_static_library_builder:humble
docker run -it --rm -v $(pwd):/project microros/micro_ros_static_library_builder:humble
