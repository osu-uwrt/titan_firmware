cmake_minimum_required(VERSION 3.13)

# The Target Board
# These can be found in lib/titan_boards/include/boards or be set to pico
# Is is preferred to define pins in the board header file and refer to that name rather than hardcoding names
# This also impacts the upload_tool for automatic device selection
set(PICO_BOARD talos/poac_board)

set(UWRT_ROBOT talos)

# Include global definitions
execute_process(COMMAND git rev-parse --show-cdup
    WORKING_DIRECTORY "${CMAKE_CURRENT_LIST_DIR}"
    OUTPUT_VARIABLE relative_path_to_repository_root OUTPUT_STRIP_TRAILING_WHITESPACE ERROR_STRIP_TRAILING_WHITESPACE)
get_filename_component(REPO_DIR "${CMAKE_CURRENT_LIST_DIR}/${relative_path_to_repository_root}" ABSOLUTE)
include(${REPO_DIR}/global_defs.cmake)

# Define project
project(dynamixel_test C CXX ASM)
titan_firmware_init()

# Define executable
add_executable(dynamixel_test
    main.c
)

# Set version: major minor (see titan/version.h for more info)
titan_generate_version_tag(dynamixel_test 1 0)

# Configure pico-sdk
pico_enable_stdio_usb(dynamixel_test 0)
pico_enable_stdio_uart(dynamixel_test 1)
pico_set_program_name(dynamixel_test "Dynamixel Test")

# Enable the 'make upload' and 'make info' commands for project
titan_use_upload_tool(dynamixel_test)
#titan_enable_bootloader(dynamixel_test can)

# Define linking and targets
pico_add_extra_outputs(dynamixel_test)
target_link_libraries(dynamixel_test PUBLIC pico_stdlib driver_dynamixel)
