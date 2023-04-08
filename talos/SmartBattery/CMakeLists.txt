cmake_minimum_required(VERSION 3.13)

# The Target Board
# These can be found in lib/uwrt_boards/include/boards or be set to pico
# Is is preferred to define pins in the board header file and refer to that name rather than hardcoding names
# This also impacts the upload_tool for automatic device selection
set(PICO_BOARD sbh_mcu)

# The Target Robot
# These can be found in lib/uwrt_boards/include/robots
# Store any robot dependent configuration values in these files
set(UWRT_ROBOT talos)

# Default Target build type - Can be overidden by passing -DCMAKE_BUILD_TYPE="Debug" in the cmake command
# Defaults to RelWithDebInfo when building normally (Compile for release, but keep symbol info for debugging)
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

# Define Project
project(smart_battery C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
pico_sdk_init()

# Define executable
file(GLOB_RECURSE SOURCES RELATIVE ${CMAKE_SOURCE_DIR} "src/*.c")
add_executable(smart_battery
	${SOURCES}
)
target_compile_options(smart_battery PRIVATE -Wall -Wextra -Wno-format)
target_include_directories(smart_battery PUBLIC include)
pico_generate_pio_header(smart_battery ${CMAKE_CURRENT_LIST_DIR}/src/i2c.pio)

# Set version: major minor release_type (see build_version.h for more info)
# Release Types: PROTO, DEV, STABLE
generate_version_tag(smart_battery 1 0 PROTO)

# Configure pico-sdk
pico_enable_stdio_uart(smart_battery 1)
pico_set_program_name(smart_battery "Smart Battery Firmware")

# Enable the 'make upload' and 'make serial' commands for project
uwrt_use_upload_tool(smart_battery)
# uwrt_enable_bootloader(smart_battery can)

# Define linking and targets
pico_add_extra_outputs(smart_battery)
target_link_libraries(smart_battery
	basic_logger	# A nice little logging library
	safety			# Handles critical code (Watchdog, Kill Switch, Fault Reporting)
	micro_ros_can	# Micro ros suppot (supports _can, _usb, _eth)
	hardware_pio
	hardware_i2c
	pico_stdlib		# Pico SDK Standard Library
	hardware_pwm	# LED PWM Support
	# Add any additional hardware here (ex. hardware_i2c)
)

