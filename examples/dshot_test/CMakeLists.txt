cmake_minimum_required(VERSION 3.13)

# The Target Board
# These can be found in lib/uwrt_boards/include/boards or be set to pico
# Is is preferred to define pins in the board header file and refer to that name rather than hardcoding names
# This also impacts the upload_tool for automatic device selection
set(PICO_BOARD mk2/puddles_backplane)

# Include global definitions
execute_process(COMMAND git rev-parse --show-cdup
	WORKING_DIRECTORY "${CMAKE_CURRENT_LIST_DIR}"
	OUTPUT_VARIABLE relative_path_to_repository_root OUTPUT_STRIP_TRAILING_WHITESPACE ERROR_STRIP_TRAILING_WHITESPACE)
get_filename_component(REPO_DIR "${CMAKE_CURRENT_LIST_DIR}/${relative_path_to_repository_root}" ABSOLUTE)
include(${REPO_DIR}/global_defs.cmake)

# Define project
project(dshot_test C CXX ASM)
titan_firmware_init()

# Define executable
add_executable(dshot_test
	main.c
)
pico_generate_pio_header(dshot_test ${CMAKE_CURRENT_LIST_DIR}/bidir_dshot.pio)

# Set version: major minor (see titan/version.h for more info)
titan_generate_version_tag(dshot_test 1 0)

# Configure pico-sdk
pico_enable_stdio_usb(dshot_test 1)
pico_enable_stdio_uart(dshot_test 0)
pico_set_program_name(dshot_test "DShot Test")

# Wait for 2 seconds for serial to connect
target_compile_definitions(dshot_test PUBLIC PICO_STDIO_USB_CONNECT_WAIT_TIMEOUT_MS=2000)

# Enable the 'make upload' and 'make serial' commands for project
titan_use_upload_tool(dshot_test)

# Define linking and targets
pico_add_extra_outputs(dshot_test)
target_link_libraries(dshot_test PUBLIC
	pico_stdlib
	hardware_pio
)
