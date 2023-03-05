cmake_minimum_required(VERSION 3.13)

# The Target Board
# These can be found in lib/uwrt_boards/include/boards or be set to pico
# Is is preferred to define pins in the board header file and refer to that name rather than hardcoding names
# This also impacts the upload_tool for automatic device selection
set(PICO_BOARD mk2_power_board)

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
project(bootloader_common C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
pico_sdk_init()

# Define executable
#file(GLOB_RECURSE SOURCES RELATIVE ${CMAKE_SOURCE_DIR} "src/*.c")
add_executable(bootloader_common
	main.c
	crt0.S
)
target_compile_options(bootloader_common PRIVATE -Wall -Wextra -Wno-format)
target_include_directories(bootloader_common PUBLIC include)

target_compile_definitions(bootloader_common PUBLIC PICO_NO_BINARY_INFO=1)


# Linking and early initialization
# Required since we want to have a custom boot2, memory map, and crt0, so we can't use pico_standard_link
# Instead we get to reimplement it all here

# Configure to use our custom boot2 to copyout bootloader into XIP
# Required so that the entire executable can live in RAM so the bootloader can *in theory* be updated
pico_define_boot_stage2(bs2_bootloader_03_copyout ${CMAKE_CURRENT_LIST_DIR}/boot2_bootloader_03_copyout.S)

# Configure the linker using options we want from pico_standard_link
pico_add_map_output(bootloader_common)
target_link_options(bootloader_common PUBLIC "LINKER:--script=${CMAKE_CURRENT_LIST_DIR}/memmap_bootloader.ld")
target_link_options(bootloader_common PUBLIC "LINKER:-z,max-page-size=4096")
target_compile_options(bootloader_common PUBLIC -ffunction-sections -fdata-sections)
target_link_options(bootloader_common PUBLIC "LINKER:--gc-sections")


# Set version: major minor release_type (see build_version.h for more info)
# Release Types: PROTO, DEV, STABLE
#generate_version_tag(bootloader_common 1 0 DEV)

# Configure pico-sdk
#pico_enable_stdio_uart(bootloader_common 1)
#pico_set_program_name(bootloader_common "Basic Bootloader")

# Enable the 'make upload' and 'make serial' commands for project
#uwrt_use_upload_tool(bootloader_common)

# Define linking and targets
pico_add_extra_outputs(bootloader_common)
target_link_libraries(bootloader_common
	pico_base_headers	# Minimum headers required for pico sdk project
	pico_bootrom		# Required for bootrom function lookups
	bs2_bootloader_03_copyout_library
)

