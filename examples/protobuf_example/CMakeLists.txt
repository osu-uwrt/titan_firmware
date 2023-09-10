cmake_minimum_required(VERSION 3.13)

# The Target Board
# These can be found in lib/uwrt_boards/include/boards or be set to pico
# Is is preferred to define pins in the board header file and refer to that name rather than hardcoding names
# This also impacts the upload_tool for automatic device selection
set(PICO_BOARD pico)

# The Target Robot
# These can be found in lib/uwrt_boards/include/robots
# Store any robot dependent configuration values in these files
set(UWRT_ROBOT talos)

# Include global definitions
execute_process(COMMAND git rev-parse --show-cdup
    WORKING_DIRECTORY "${CMAKE_CURRENT_LIST_DIR}"
    OUTPUT_VARIABLE relative_path_to_repository_root OUTPUT_STRIP_TRAILING_WHITESPACE ERROR_STRIP_TRAILING_WHITESPACE)
get_filename_component(REPO_DIR "${CMAKE_CURRENT_LIST_DIR}/${relative_path_to_repository_root}" ABSOLUTE)
include(${REPO_DIR}/global_defs.cmake)

# Define project
project(protobuf_example C CXX ASM)
titan_firmware_init()

# Setup nanopb
include_directories(${NANOPB_INCLUDE_DIRS})
include_directories(${CMAKE_CURRENT_BINARY_DIR})
NANOPB_GENERATE_CPP(PROTO_SRCS PROTO_HDRS protos.proto)

# Define executable
add_executable(protobuf_example main.c ${PROTO_HDRS} ${PROTO_SRCS})

# Set version: major minor (see titan/version.h for more info)
titan_generate_version_tag(protobuf_example 1 0)

# Configure pico-sdk
pico_enable_stdio_usb(protobuf_example 1)
pico_enable_stdio_uart(protobuf_example 0)
pico_set_program_name(protobuf_example "Protobuf Example")

# Wait for 2 seconds for serial to connect
target_compile_definitions(protobuf_example PUBLIC PICO_STDIO_USB_CONNECT_WAIT_TIMEOUT_MS=2000)

# Enable the 'make upload' and 'make serial' commands for project
titan_use_upload_tool(protobuf_example)
#titan_enable_bootloader(pico_sdk_example can)

# Define linking and targets
pico_add_extra_outputs(protobuf_example)
target_link_libraries(protobuf_example PUBLIC pico_stdlib)
