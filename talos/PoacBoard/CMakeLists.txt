cmake_minimum_required(VERSION 3.13)

# The Target Board
# These can be found in lib/titan_boards/include/boards or be set to pico
# Is is preferred to define pins in the board header file and refer to that name rather than hardcoding names
# This also impacts the upload_tool for automatic device selection
set(PICO_BOARD talos/poac_board)

# The Target Robot
# These can be found in lib/titan_boards/include/robots
# Store any robot dependent configuration values in these files
set(UWRT_ROBOT talos)

# Include global definitions
execute_process(COMMAND git rev-parse --show-cdup
    WORKING_DIRECTORY "${CMAKE_CURRENT_LIST_DIR}"
    OUTPUT_VARIABLE relative_path_to_repository_root OUTPUT_STRIP_TRAILING_WHITESPACE ERROR_STRIP_TRAILING_WHITESPACE)
get_filename_component(REPO_DIR "${CMAKE_CURRENT_LIST_DIR}/${relative_path_to_repository_root}" ABSOLUTE)
include(${REPO_DIR}/global_defs.cmake)

# Define Project
project(poac_board C CXX ASM)
titan_firmware_init()

# Define executable
file(GLOB_RECURSE SOURCES RELATIVE ${CMAKE_SOURCE_DIR} "src/*.c")
add_executable(poac_board
    ${SOURCES}
)
target_include_directories(poac_board PUBLIC include)

# Required so we can register our own gpio interrupt for the kill switch (only one gpio interrupt supported)
target_compile_definitions(poac_board PUBLIC CAN_MCP251XFD_USE_EXTERNAL_INTERRUPT_CB=1)

# Set version: major minor (see titan/version.h for more info)
titan_generate_version_tag(poac_board 1 0)

# Configure pico-sdk
pico_enable_stdio_uart(poac_board 1)
pico_set_program_name(poac_board "Poac Board FW")

# Enable the 'make upload' and 'make info' commands for project
titan_use_upload_tool(poac_board)
titan_enable_bootloader(poac_board can)

# Define linking and targets
pico_add_extra_outputs(poac_board)
target_link_libraries(poac_board PUBLIC
    driver_led          # Status LED support
    titan_logger        # A nice little logging library
    titan_safety        # Handles critical code (Watchdog, Kill Switch, Fault Reporting)
    micro_ros_can       # Micro ros suppot (supports _can, _usb, _eth)

    pico_stdlib         # Pico SDK Standard Library
    hardware_adc
    driver_async_i2c
    driver_dynamixel
    driver_mcp3426
    driver_sht41
)

