cmake_minimum_required(VERSION 3.13)

# The Target Board
# These can be found in lib/uwrt_boards/include/boards or be set to pico
# Is is preferred to define pins in the board header file and refer to that name rather than hardcoding names
# This also impacts the upload_tool for automatic device selection
set(PICO_BOARD backplane_breakout_board_eval_hat)

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

# Define executable
add_executable(copro_firmware
	src/main.c
	src/drivers/adc.c
	src/drivers/async_i2c.c
	src/drivers/memmonitor.c
	src/drivers/safety.c
	src/drivers/safety_helper.S
	src/hw/balancer_adc.c
	src/hw/depth_sensor.c
	src/hw/dio.c
#	src/hw/dshot.c
	src/hw/esc_pwm.c
	src/hw/esc_adc.c
	src/tasks/cooling.c
	src/tasks/lowbatt.c
	src/tasks/ros.c
)

#pico_generate_pio_header(copro_firmware ${CMAKE_CURRENT_LIST_DIR}/src/hw/dshot.pio)

target_compile_options(copro_firmware PRIVATE -Wall -Wextra -Wno-format -Werror)
target_include_directories(copro_firmware PUBLIC include)

# Set version: major minor release_type (see build_version.h for more info)
# Release Types: PROTO, DEV, STABLE
generate_version_tag(copro_firmware 1 0 DEV)

# Configure pico-sdk
pico_enable_stdio_uart(copro_firmware 0)
pico_set_program_name(copro_firmware "Copro Firmware")
target_compile_definitions(copro_firmware PUBLIC USB_PRODUCT_STR="Coprocessor")

# Enable the 'make upload' and 'make serial' commands for project
uwrt_use_upload_tool(copro_firmware)

# Enable microROS
micro_ros_enable(copro_firmware)

target_compile_definitions(copro_firmware PUBLIC PICO_PANIC_FUNCTION=safety_panic)
target_compile_definitions(copro_firmware PUBLIC PARAM_ASSERTIONS_ENABLE_ALL=1)

# Define linking and targets
pico_add_extra_outputs(copro_firmware)
target_link_libraries(copro_firmware
	pico_stdlib
	hardware_exception
#	hardware_pio
	hardware_pwm
	hardware_i2c
	# Hook required functions for safety watchdog reporting
	"-Wl,--wrap=__assert_func,--wrap=uxr_run_session_until_confirm_delivery,--wrap=uxr_ping_agent_session"
)
