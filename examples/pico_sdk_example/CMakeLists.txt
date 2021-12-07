cmake_minimum_required(VERSION 3.13)

# The Target Board
# These can be found in lib/uwrt_boards/include/boards or be set to pico
# Is is preferred to define pins in the board header file and refer to that name rather than hardcoding names
# This also impacts the upload_tool for automatic device selection
set(PICO_BOARD pico)

# Target build type
# Setting to Debug increases size but gives more useful information during debugging
# Set to Debug if debugging with gdb
set(CMAKE_BUILD_TYPE Release)

# Include global definitions
execute_process(COMMAND git rev-parse --show-cdup
	WORKING_DIRECTORY "${CMAKE_CURRENT_LIST_DIR}"
	OUTPUT_VARIABLE relative_path_to_repository_root OUTPUT_STRIP_TRAILING_WHITESPACE ERROR_STRIP_TRAILING_WHITESPACE)
get_filename_component(REPO_DIR "${CMAKE_CURRENT_LIST_DIR}/${relative_path_to_repository_root}" ABSOLUTE)
include(${REPO_DIR}/global_defs.cmake)

project(pico_sdk_example C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
pico_sdk_init()

# Define executable
add_executable(pico_sdk_example
	main.c
)

# Set version: major minor
generate_version_tag(pico_sdk_example 1 0)

# Configure pico-sdk
pico_enable_stdio_usb(pico_sdk_example 1)
pico_enable_stdio_uart(pico_sdk_example 0)
pico_set_program_name(pico_sdk_example "Pico SDK Example")

# Wait for 2 seconds for serial to connect
target_compile_definitions(pico_sdk_example PUBLIC PICO_STDIO_USB_CONNECT_WAIT_TIMEOUT_MS=2000)

# Enable the 'make upload' and 'make serial' commands for project
uwrt_use_upload_tool(pico_sdk_example)

# Define linking and targets
pico_add_extra_outputs(pico_sdk_example)
target_link_libraries(pico_sdk_example pico_stdlib)
