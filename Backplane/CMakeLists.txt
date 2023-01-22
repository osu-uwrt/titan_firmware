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

project(backplane_test C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
pico_sdk_init()

# Setup wiznet

# Set ethernet chip
set(WIZNET_CHIP W5200)

if(${WIZNET_CHIP} STREQUAL W5100S)
    add_definitions(-D_WIZCHIP_=W5100S)
elseif(${WIZNET_CHIP} STREQUAL W5500)
    add_definitions(-D_WIZCHIP_=W5500)
elseif(${WIZNET_CHIP} STREQUAL W5200)
    add_definitions(-D_WIZCHIP_=W5200)
else()
    message(FATAL_ERROR "WIZNET_CHIP is wrong = ${WIZNET_CHIP}")
endif()

message(STATUS "WIZNET_CHIP = ${WIZNET_CHIP}")

# Set the project root directory if it's not already defined, as may happen if
# the tests folder is included directly by a parent project, without including
# the top level CMakeLists.txt.
if(NOT DEFINED WIZNET_DIR)
    set(WIZNET_DIR ${REPO_DIR}/lib/ioLibrary_Driver)
    message(STATUS "WIZNET_DIR = ${WIZNET_DIR}")
endif()


if(NOT DEFINED PORT_DIR)
    set(PORT_DIR ${REPO_DIR}/lib/ioLibrary_Port)
    message(STATUS "PORT_DIR = ${PORT_DIR}")
endif()

include(${REPO_DIR}/lib/ioLibrary_import.cmake)
add_subdirectory(${PORT_DIR} ioLibrary_port)

# Define executable
add_executable(pico_sdk_example
	main.c
)
# pico_generate_pio_header(copro_firmware ${CMAKE_CURRENT_LIST_DIR}/src/hw/dshot.pio)

target_compile_options(copro_firmware PRIVATE -Wall -Wextra -Wno-format)
target_include_directories(copro_firmware PUBLIC include)

# Set version: major minor release_type (see build_version.h for more info)
generate_version_tag(pico_sdk_example 1 0 DEV) # Release Types: PROTO, DEV, STABLE

# Configure pico-sdk
pico_enable_stdio_usb(pico_sdk_example 1)
pico_set_program_name(pico_sdk_example "Backplane Test")

# Wait for 2 seconds for serial to connect
target_compile_definitions(pico_sdk_example PUBLIC PICO_STDIO_USB_CONNECT_WAIT_TIMEOUT_MS=2000)

# Enable the 'make upload' and 'make serial' commands for project
uwrt_use_upload_tool(pico_sdk_example)

# Define linking and targets
pico_add_extra_outputs(pico_sdk_example)
target_link_libraries(pico_sdk_example 
	pico_stdlib
	hardware_pio
	hardware_pwm
	hardware_i2c
	hardware_adc
	hardware_spi
	hardware_dma
	
	ETHERNET_FILES
	IOLIBRARY_FILES
)
