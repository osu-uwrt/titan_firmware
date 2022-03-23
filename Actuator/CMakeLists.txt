cmake_minimum_required(VERSION 3.13)

# The Target Board
# These can be found in lib/uwrt_boards/include/boards or be set to pico
# Is is preferred to define pins in the board header file and refer to that name rather than hardcoding names
# This also impacts the upload_tool for automatic device selection
set(PICO_BOARD rp2040_eval_board)

# Target build type
# Setting to Debug increases size but gives more useful information during debugging
# Set to Debug if debugging with gdb
set(CMAKE_BUILD_TYPE Debug)

# Include global definitions
execute_process(COMMAND git rev-parse --show-cdup
	WORKING_DIRECTORY "${CMAKE_CURRENT_LIST_DIR}"
	OUTPUT_VARIABLE relative_path_to_repository_root OUTPUT_STRIP_TRAILING_WHITESPACE ERROR_STRIP_TRAILING_WHITESPACE)
get_filename_component(REPO_DIR "${CMAKE_CURRENT_LIST_DIR}/${relative_path_to_repository_root}" ABSOLUTE)
include(${REPO_DIR}/global_defs.cmake)

project(actuator_firmware C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
pico_sdk_init()

# Define executable
add_executable(actuator_firmware
	main.c
	async_i2c_target.c
)

target_compile_definitions(actuator_firmware PUBLIC PARAM_ASSERTIONS_ENABLE_ALL=1)

# Set version: major minor release_type (see build_version.h for more info)
# Release Types: PROTO, DEV, STABLE
generate_version_tag(actuator_firmware 1 0 PROTO)

# Configure pico-sdk
pico_enable_stdio_uart(actuator_firmware 0)
pico_set_program_name(actuator_firmware "Actuator Firmware")
uwrt_enable_dual_uart(actuator_firmware)

# Enable the 'make upload' and 'make serial' commands for project
uwrt_use_upload_tool(actuator_firmware)

# Include the actuator i2c interafce headers into this project
add_subdirectory(${REPO_DIR}/lib/actuator_i2c_interface/ actuator_i2c_interface)
add_subdirectory(${REPO_DIR}/lib/basic_logger/ basic_logger)
target_compile_definitions(actuator_firmware PUBLIC BASIC_LOGGER_DEFAULT_LEVEL=LEVEL_INFO)
target_compile_definitions(actuator_firmware PUBLIC BASIC_LOGGER_PRINT_SOURCE_LOCATION=0)

# Define linking and targets
pico_add_extra_outputs(actuator_firmware)
target_link_libraries(actuator_firmware 
	actuator_i2c_interface
	basic_logging

	pico_stdlib
	hardware_i2c
	hardware_watchdog
)
