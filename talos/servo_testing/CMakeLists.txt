cmake_minimum_required(VERSION 3.13)

# The Target Board
# These can be found in lib/titan_boards/include/boards or be set to pico
# For example, if you want to use lib/titan_boards/include/boards/mk2/esc_board.h, set this to mk2/esc_board
# Is is preferred to define pins in the board header file and refer to that name rather than hardcoding names
# This also impacts the upload_tool for automatic device selection
set(PICO_BOARD pico)

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

# Define project
project(pico_sdk_example C CXX ASM)
titan_firmware_init()

# Define executable
add_executable(pico_sdk_example
    main.c
    async_uart.c
)

# Set version: major minor (see titan/version.h for more info)
titan_generate_version_tag(pico_sdk_example 1 0)

# Configure pico-sdk
pico_enable_stdio_usb(pico_sdk_example 1)
pico_enable_stdio_uart(pico_sdk_example 0)
pico_set_program_name(pico_sdk_example "Pico SDK Example")

pico_generate_pio_header(pico_sdk_example ${CMAKE_CURRENT_LIST_DIR}/uart_multidrop.pio)

# Wait for 2 seconds for serial to connect
target_compile_definitions(pico_sdk_example PUBLIC PICO_STDIO_USB_CONNECT_WAIT_TIMEOUT_MS=2000)

# Enable the 'make upload' and 'make info' commands for project
titan_use_upload_tool(pico_sdk_example)
#titan_enable_bootloader(pico_sdk_example can)

# Define linking and targets
pico_add_extra_outputs(pico_sdk_example)
target_link_libraries(pico_sdk_example PUBLIC
    pico_stdlib
    hardware_dma
    hardware_pio
    titan_logger
    titan_queue
)
