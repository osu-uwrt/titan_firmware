cmake_minimum_required(VERSION 3.13)

# The Target Board
# These can be found in lib/uwrt_boards/include/boards or be set to pico
# Is is preferred to define pins in the board header file and refer to that name rather than hardcoding names
# This also impacts the upload_tool for automatic device selection
set(PICO_BOARD mk2/puddles_backplane)

# The Target Robot
# These can be found in lib/uwrt_boards/include/robots
# Store any robot dependent configuration values in these files
set(UWRT_ROBOT puddles)

# Include global definitions
execute_process(COMMAND git rev-parse --show-cdup
    WORKING_DIRECTORY "${CMAKE_CURRENT_LIST_DIR}"
    OUTPUT_VARIABLE relative_path_to_repository_root OUTPUT_STRIP_TRAILING_WHITESPACE ERROR_STRIP_TRAILING_WHITESPACE)
get_filename_component(REPO_DIR "${CMAKE_CURRENT_LIST_DIR}/${relative_path_to_repository_root}" ABSOLUTE)
include(${REPO_DIR}/global_defs.cmake)

# Define Project
project(backplane_firmware C CXX ASM)
titan_firmware_init()

# Define executable
file(GLOB_RECURSE SOURCES RELATIVE ${CMAKE_SOURCE_DIR} "src/*.c")
add_executable(backplane_firmware
    ${SOURCES}
)
pico_generate_pio_header(backplane_firmware ${CMAKE_CURRENT_LIST_DIR}/src/bidir_dshot.pio)
target_include_directories(backplane_firmware PUBLIC include)

# Set version: major minor (see titan/version.h for more info)
titan_generate_version_tag(backplane_firmware 1 0)

# Configure pico-sdk
pico_enable_stdio_uart(backplane_firmware 1)
pico_set_program_name(backplane_firmware "Puddles Backplane Firmware")

# Wait for 2 seconds for serial to connect
target_compile_definitions(backplane_firmware PUBLIC PICO_STDIO_USB_CONNECT_WAIT_TIMEOUT_MS=2000)

# Enable the 'make upload' and 'make serial' commands for project
titan_use_upload_tool(backplane_firmware)
titan_enable_bootloader(backplane_firmware eth)

# Compile Definitions
target_compile_definitions(backplane_firmware PUBLIC SAFETY_ENABLE_PROFILER=1)

# Define linking and targets
pico_add_extra_outputs(backplane_firmware)
target_link_libraries(backplane_firmware PUBLIC
    driver_async_i2c
    driver_depth
    driver_led
    driver_mcp3426
    hardware_pio
    micro_ros_eth
    pico_stdlib
    titan_logger
    titan_safety
)
