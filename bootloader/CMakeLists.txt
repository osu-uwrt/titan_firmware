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

# Include the runtime subdirectory
add_subdirectory(bootloader_runtime)

# Define executable
#file(GLOB_RECURSE SOURCES RELATIVE ${CMAKE_SOURCE_DIR} "src/*.c")
add_executable(bootloader_common
	main.c
)
target_compile_options(bootloader_common PRIVATE -Wall -Wextra -Wno-format)
target_include_directories(bootloader_common PUBLIC include)

# Set version: major minor release_type (see build_version.h for more info)
# Release Types: PROTO, DEV, STABLE
#generate_version_tag(bootloader_common 1 0 DEV)

# Enable the 'make upload' and 'make serial' commands for project
#uwrt_use_upload_tool(bootloader_common)

# Define linking and targets
pico_add_extra_outputs(bootloader_common)
target_link_libraries(bootloader_common
	bootloader_runtime
)

