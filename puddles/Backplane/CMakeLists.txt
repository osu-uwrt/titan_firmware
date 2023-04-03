cmake_minimum_required(VERSION 3.13)

# The Target Board
# These can be found in lib/uwrt_boards/include/boards or be set to pico
# Is is preferred to define pins in the board header file and refer to that name rather than hardcoding names
# This also impacts the upload_tool for automatic device selection
set(PICO_BOARD puddles_backplane)

# The Target Robot
# These can be found in lib/uwrt_boards/include/robots
# Store any robot dependent configuration values in these files
set(UWRT_ROBOT puddles)

# Default Target build type - Can be overidden by passing -DCMAKE_BUILD_TYPE="Debug" in the cmake command
# Defaults to RelWithDebInfo when building normally
# Overriding Debug increases size and slows down code, but gives more useful information during debugging
if(NOT CMAKE_BUILD_TYPE)
	set(CMAKE_BUILD_TYPE RelWithDebInfo)
endif()

# Include global definitions
execute_process(COMMAND git rev-parse --show-cdup
	WORKING_DIRECTORY "${CMAKE_CURRENT_LIST_DIR}"
	OUTPUT_VARIABLE relative_path_to_repository_root OUTPUT_STRIP_TRAILING_WHITESPACE ERROR_STRIP_TRAILING_WHITESPACE)
get_filename_component(REPO_DIR "${CMAKE_CURRENT_LIST_DIR}/${relative_path_to_repository_root}" ABSOLUTE)
include(${REPO_DIR}/global_defs.cmake)

project(backplane_test C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
pico_sdk_init()

# Define executable
file(GLOB_RECURSE SOURCES RELATIVE ${CMAKE_SOURCE_DIR} "src/*.c")
add_executable(backplane_firmware
	${SOURCES}
)
pico_generate_pio_header(backplane_firmware ${CMAKE_CURRENT_LIST_DIR}/src/bidir_dshot.pio)

target_compile_options(backplane_firmware PRIVATE -Wall -Wextra -Wno-format)
target_include_directories(backplane_firmware PUBLIC include)

# Set version: major minor release_type (see build_version.h for more info)
generate_version_tag(backplane_firmware 1 0 DEV) # Release Types: PROTO, DEV, STABLE

# Configure pico-sdk
pico_enable_stdio_uart(backplane_firmware 1)
pico_set_program_name(backplane_firmware "Backplane Test")

# Wait for 2 seconds for serial to connect
target_compile_definitions(backplane_firmware PUBLIC PICO_STDIO_USB_CONNECT_WAIT_TIMEOUT_MS=2000)

# Enable the 'make upload' and 'make serial' commands for project
uwrt_use_upload_tool(backplane_firmware)
uwrt_enable_bootloader(backplane_firmware eth)

# Compile Definitions
target_compile_definitions(backplane_firmware PUBLIC PARAM_ASSERTIONS_ENABLE_ALL=1)
# target_compile_definitions(backplane_firmware PUBLIC BASIC_LOGGER_MIN_SEVERITY=LEVEL_DEBUG)
# target_compile_definitions(backplane_firmware PUBLIC BASIC_LOGGER_DEFAULT_LEVEL=LEVEL_INFO)
# target_compile_definitions(backplane_firmware PUBLIC BASIC_LOGGER_PRINT_SOURCE_LOCATION=0)
# target_compile_definitions(backplane_firmware PUBLIC SAFETY_ROS_SUPPORT=1)
# target_compile_definitions(backplane_firmware PUBLIC BASIC_LOGGER_USE_COLOR_CODES=1)

# Define linking and targets
pico_add_extra_outputs(backplane_firmware)
target_link_libraries(backplane_firmware
	basic_logger
	pico_stdlib
	micro_ros_eth
	safety

	hardware_pio
	hardware_pwm
	async_i2c
)
