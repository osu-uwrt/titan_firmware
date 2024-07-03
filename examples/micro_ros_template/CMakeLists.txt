cmake_minimum_required(VERSION 3.13)

# The Target Board
# These can be found in lib/titan_boards/include/boards or be set to pico
# For example, if you want to use lib/titan_boards/include/boards/mk2/esc_board.h, set this to mk2/esc_board
# Is is preferred to define pins in the board header file and refer to that name rather than hardcoding names
# This also impacts the upload_tool for automatic device selection
set(PICO_BOARD mk2/power_board)  # TODO: Set this to your target board

# The Target Robot
# These can be found in lib/titan_boards/include/robots
# Store any robot dependent configuration values in these files
set(UWRT_ROBOT talos)  # TODO: Set this to your target robot

# Include global definitions
execute_process(COMMAND git rev-parse --show-cdup
    WORKING_DIRECTORY "${CMAKE_CURRENT_LIST_DIR}"
    OUTPUT_VARIABLE relative_path_to_repository_root OUTPUT_STRIP_TRAILING_WHITESPACE ERROR_STRIP_TRAILING_WHITESPACE)
get_filename_component(REPO_DIR "${CMAKE_CURRENT_LIST_DIR}/${relative_path_to_repository_root}" ABSOLUTE)
include(${REPO_DIR}/global_defs.cmake)

# Define Project
# TODO: Rename project and executable to your project name
project(template_project C CXX ASM)
titan_firmware_init()

# Define executable
file(GLOB_RECURSE SOURCES RELATIVE ${CMAKE_SOURCE_DIR} "src/*.c")
add_executable(template_project
    ${SOURCES}
)
target_include_directories(template_project PUBLIC include)

# Set version: major minor (see titan/version.h for more info)
titan_generate_version_tag(template_project 1 0)

# Configure pico-sdk
pico_enable_stdio_uart(template_project 1)
pico_set_program_name(template_project "Template Project")  # TODO: Set the project name

# Enable the 'make upload' and 'make info' commands for project
titan_use_upload_tool(template_project)
titan_enable_bootloader(template_project can)  # TODO: Select the same transport as below, or remove this line if using USB

# Define linking and targets
pico_add_extra_outputs(template_project)
target_link_libraries(template_project PUBLIC
    pico_stdlib     # Pico SDK Standard Library
    driver_led      # Status LED support
    # TODO: Select your transport:
    micro_ros_can   # Micro ros support (supports _can, _usb, _eth)
    titan_logger    # A nice little logging library
    titan_safety    # Handles critical code (Watchdog, Kill Switch, Fault Reporting)

    # TODO: Add any additional hardware here (ex. hardware_adc)
)

