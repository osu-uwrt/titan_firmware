cmake_minimum_required(VERSION 3.13)

# The Target Board
# These can be found in lib/titan_boards/include/boards or be set to pico
# For example, if you want to use lib/titan_boards/include/boards/mk2/esc_board.h, set this to mk2/esc_board
# Is is preferred to define pins in the board header file and refer to that name rather than hardcoding names
# This also impacts the upload_tool for automatic device selection
set(PICO_BOARD pico)  # TODO: Set this to your target board

# The Target Robot
# These can be found in lib/titan_boards/include/robots
# Store any robot dependent configuration values in these files
#set(UWRT_ROBOT talos)  # TODO: Uncommand and set this to your target robot, or remove if not targetting a vehicle

# Include global definitions
execute_process(COMMAND git rev-parse --show-cdup
    WORKING_DIRECTORY "${CMAKE_CURRENT_LIST_DIR}"
    OUTPUT_VARIABLE relative_path_to_repository_root OUTPUT_STRIP_TRAILING_WHITESPACE ERROR_STRIP_TRAILING_WHITESPACE)
get_filename_component(REPO_DIR "${CMAKE_CURRENT_LIST_DIR}/${relative_path_to_repository_root}" ABSOLUTE)
include(${REPO_DIR}/global_defs.cmake)

# Define Project
# TODO: Rename project and executable to your project name
project(nros_test C CXX ASM)
titan_firmware_init()

# Define executable
file(GLOB_RECURSE SOURCES RELATIVE ${CMAKE_SOURCE_DIR} "src/*.c")
add_executable(nros_test
    ${SOURCES}
)
target_include_directories(nros_test PUBLIC include)

add_subdirectory(
    ${CMAKE_CURRENT_SOURCE_DIR}/../../../../../projects/nano_ros/libnanoros
    ${CMAKE_CURRENT_BINARY_DIR}/libnanoros
)

# Set version: major minor (see titan/version.h for more info)
titan_generate_version_tag(nros_test 1 0)

# Configure pico-sdk
pico_enable_stdio_uart(nros_test 1)  # TODO: Disable if running on board without tagconnect or UART headers
#pico_enable_stdio_usb(nros_test 1)  # TODO: Enable if you don't have hardware uart
pico_set_program_name(nros_test "nros_test")  # TODO: Set the project name

# Enable the 'make upload' and 'make info' commands for project
titan_use_upload_tool(nros_test)
# titan_enable_bootloader(nros_test can)  # TODO: Enable and select transport if using bootloader

# Define linking and targets
pico_add_extra_outputs(nros_test)
target_link_libraries(nros_test PUBLIC
    titan_dual_stdio_usb
    pico_stdlib     # Pico SDK Standard Library
    titan_logger    # A nice little logging library
    titan_safety    # Handles critical code (Watchdog, Kill Switch, Fault Reporting)
    libnanoros

    # TODO: Add any additional hardware here (ex. hardware_adc)
)

