cmake_minimum_required(VERSION 3.13)

# The Target Board
# These can be found in lib/uwrt_boards/include/boards or be set to pico
# Is is preferred to define pins in the board header file and refer to that name rather than hardcoding names
# This also impacts the upload_tool for automatic device selection
set(PICO_BOARD mk2_actuator_board)

# The Target Robot
# These can be found in lib/uwrt_boards/include/robots
# Store any robot dependent configuration values in these files
set(UWRT_ROBOT talos)

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

# Define Project
project(actuator_mk2 C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
pico_sdk_init()

# Define executable
file(GLOB_RECURSE SOURCES RELATIVE ${CMAKE_SOURCE_DIR} "src/*.c")
add_executable(actuator_mk2
	${SOURCES}
)
pico_generate_pio_header(actuator_mk2 ${CMAKE_CURRENT_LIST_DIR}/src/actuators/torpedo.pio)
target_compile_definitions(actuator_mk2 PUBLIC PARAM_ASSERTIONS_ENABLED_LIFETIME_CHECK=1)

target_compile_options(actuator_mk2 PRIVATE -Wall -Wextra -Wno-format)
target_compile_definitions(actuator_mk2 PUBLIC PARAM_ASSERTIONS_ENABLE_ALL=1)
target_include_directories(actuator_mk2 PUBLIC include)

# Set version: major minor release_type (see build_version.h for more info)
# Release Types: PROTO, DEV, STABLE
generate_version_tag(actuator_mk2 1 0 PROTO)

# Configure pico-sdk
pico_enable_stdio_uart(actuator_mk2 1)
pico_set_program_name(actuator_mk2 "Template Project")

# Enable the 'make upload' and 'make serial' commands for project
uwrt_use_upload_tool(actuator_mk2)

# Define linking and targets
pico_add_extra_outputs(actuator_mk2)
target_link_libraries(actuator_mk2
	basic_logger	# A nice little logging library
	safety			# Handles critical code (Watchdog, Kill Switch, Fault Reporting)
	micro_ros_can	# Micro ros suppot (supports _can, _usb, _eth)

	pico_stdlib		# Pico SDK Standard Library
	hardware_adc
	hardware_pio
	# TODO: do we need watchdog here? 
	# Add any additional hardware here (ex. hardware_i2c)
)

