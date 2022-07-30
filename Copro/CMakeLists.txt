cmake_minimum_required(VERSION 3.13)

# The Target Board
# These can be found in lib/uwrt_boards/include/boards or be set to pico
# Is is preferred to define pins in the board header file and refer to that name rather than hardcoding names
# This also impacts the upload_tool for automatic device selection
set(PICO_BOARD backplane_breakout_board)

# The Target Robot
# These can be found in lib/uwrt_boards/include/robots
# Store any robot dependent configuration values in these files
set(UWRT_ROBOT tempest)

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

project(copro_firmware C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
pico_sdk_init()

# Setup wiznet

# Set ethernet chip
set(WIZNET_CHIP W5500)

if(${WIZNET_CHIP} STREQUAL W5100S)
    add_definitions(-D_WIZCHIP_=W5100S)
elseif(${WIZNET_CHIP} STREQUAL W5500)
    add_definitions(-D_WIZCHIP_=W5500)
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
file(GLOB_RECURSE SOURCES RELATIVE ${CMAKE_SOURCE_DIR} "src/*.c")
add_executable(copro_firmware
	${SOURCES}
)
pico_generate_pio_header(copro_firmware ${CMAKE_CURRENT_LIST_DIR}/src/hw/dshot.pio)

target_compile_options(copro_firmware PRIVATE -Wall -Wextra -Wno-format)
target_include_directories(copro_firmware PUBLIC include)

# Set version: major minor release_type (see build_version.h for more info)
generate_version_tag(copro_firmware 1 0 DEV)  # Release Types: PROTO, DEV, STABLE

# Configure pico-sdk
pico_enable_stdio_uart(copro_firmware 0)
pico_set_program_name(copro_firmware "Copro Firmware")
target_compile_definitions(copro_firmware PUBLIC USB_PRODUCT_STR="Coprocessor")

# Enable the 'make upload' and 'make serial' commands for project
uwrt_use_upload_tool(copro_firmware)

# Enable micro-ROS
micro_ros_enable(copro_firmware)

# Compile Definitions
target_compile_definitions(copro_firmware PUBLIC PARAM_ASSERTIONS_ENABLE_ALL=1)
target_compile_definitions(copro_firmware PUBLIC BASIC_LOGGER_MIN_SEVERITY=LEVEL_DEBUG)
target_compile_definitions(copro_firmware PUBLIC BASIC_LOGGER_DEFAULT_LEVEL=LEVEL_INFO)
target_compile_definitions(copro_firmware PUBLIC BASIC_LOGGER_PRINT_SOURCE_LOCATION=0)
target_compile_definitions(copro_firmware PUBLIC SAFETY_ROS_SUPPORT=1)
target_compile_definitions(copro_firmware PUBLIC BASIC_LOGGER_USE_COLOR_CODES=1)

# Define linking and targets
pico_add_extra_outputs(copro_firmware)
target_link_libraries(copro_firmware
	actuator_i2c_interface
	basic_logger
	safety

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
