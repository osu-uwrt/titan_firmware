cmake_minimum_required(VERSION 3.13)

# The Target Board
# These can be found in lib/uwrt_boards/include/boards or be set to pico
# Is is preferred to define pins in the board header file and refer to that name rather than hardcoding names
# This also impacts the upload_tool for automatic device selection
set(PICO_BOARD mk1/backplane_breakout_board)

# Include global definitions
execute_process(COMMAND git rev-parse --show-cdup
	WORKING_DIRECTORY "${CMAKE_CURRENT_LIST_DIR}"
	OUTPUT_VARIABLE relative_path_to_repository_root OUTPUT_STRIP_TRAILING_WHITESPACE ERROR_STRIP_TRAILING_WHITESPACE)
get_filename_component(REPO_DIR "${CMAKE_CURRENT_LIST_DIR}/${relative_path_to_repository_root}" ABSOLUTE)
include(${REPO_DIR}/global_defs.cmake)

# Define project
project(micro_ros_example C CXX ASM)
titan_firmware_init()

# Define executable
add_executable(micro_ros_example
	main.c
	safety_interface.c
)

# Set version: major minor (see titan/version.h for more info)
titan_generate_version_tag(micro_ros_example 1 0)

# Configure pico-sdk
pico_enable_stdio_uart(micro_ros_example 0)
pico_enable_stdio_usb(micro_ros_example 1)
pico_set_program_name(micro_ros_example "Micro-ROS Example")

# Enable the 'make upload' and 'make serial' commands for project
titan_use_upload_tool(micro_ros_example)

# Define linking and targets
pico_add_extra_outputs(micro_ros_example)
target_link_libraries(micro_ros_example PUBLIC pico_stdlib micro_ros_can titan_safety)
