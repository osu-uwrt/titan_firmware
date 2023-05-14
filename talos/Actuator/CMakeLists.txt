cmake_minimum_required(VERSION 3.13)

# The Target Board
# These can be found in lib/uwrt_boards/include/boards or be set to pico
# Is is preferred to define pins in the board header file and refer to that name rather than hardcoding names
# This also impacts the upload_tool for automatic device selection
set(PICO_BOARD mk2/actuator_board)

# The Target Robot
# These can be found in lib/uwrt_boards/include/robots
# Store any robot dependent configuration values in these files
set(UWRT_ROBOT talos)

# Configure whether we're using new or old actuators
set(USE_V2_ACTUATORS 1 CACHE STRING "Enables new actuators")

# Include global definitions
execute_process(COMMAND git rev-parse --show-cdup
	WORKING_DIRECTORY "${CMAKE_CURRENT_LIST_DIR}"
	OUTPUT_VARIABLE relative_path_to_repository_root OUTPUT_STRIP_TRAILING_WHITESPACE ERROR_STRIP_TRAILING_WHITESPACE)
get_filename_component(REPO_DIR "${CMAKE_CURRENT_LIST_DIR}/${relative_path_to_repository_root}" ABSOLUTE)
include(${REPO_DIR}/global_defs.cmake)

# Define Project
project(actuator_mk2 C CXX ASM)
titan_firmware_init()

# Define executable
# Can't use glob since glob doesn't like to ignore specific directories
add_executable(actuator_mk2
	src/main.c
	src/safety_interface.c
	src/ros/ros.c
	src/ros/ros_actuators.c
)
target_include_directories(actuator_mk2 PUBLIC include)

# Set version: major minor (see titan/version.h for more info)
titan_generate_version_tag(actuator_mk2 1 0)

# Configure pico-sdk
pico_enable_stdio_uart(actuator_mk2 1)
pico_set_program_name(actuator_mk2 "Actuator Board Firmware")

# Enable the 'make upload' and 'make serial' commands for project
titan_use_upload_tool(actuator_mk2)
titan_enable_bootloader(actuator_mk2 can)

# Define linking and targets
pico_add_extra_outputs(actuator_mk2)
target_link_libraries(actuator_mk2 PUBLIC
	driver_led			# Status LED Support
	titan_logger		# A nice little logging library
	titan_safety		# Handles critical code (Watchdog, Kill Switch, Fault Reporting)
	micro_ros_can		# Micro ros suppot (supports _can, _usb, _eth)
	driver_status_strip	# Top LED stip

	pico_stdlib			# Pico SDK Standard Library
)

# Configure target depending on if running Mark 1 or Mark 2 Actuators
if (USE_V2_ACTUATORS)
	message("Building with Actuator V2 Support")
	target_compile_definitions(actuator_mk2 PUBLIC ACTUATOR_V2_SUPPORT=1)

	# List of sources files in mark 1 directory
	set(ACTUATORS_V2_SPECIFIC_FILES
		actuators.c
		torpedo_marker.c
		claw_disabled.c
	)

	# Append actuators v1 and target all the sources
	list(TRANSFORM ACTUATORS_V2_SPECIFIC_FILES PREPEND ${CMAKE_CURRENT_SOURCE_DIR}/src/actuators_v2/)
	target_sources(actuator_mk2 PUBLIC
		${ACTUATORS_V2_SPECIFIC_FILES}
		src/ros/actuator_v2_dynamixel.c
	)

	# Link the required libraries for mark 1 actuators
	target_link_libraries(actuator_mk2 PUBLIC
		driver_dynamixel
	)
else()
	message("Building with Actuator V1 Support")
	target_compile_definitions(actuator_mk2 PUBLIC ACTUATOR_V1_SUPPORT=1)

	# List of sources files in mark 1 directory
	set(ACTUATORS_V1_SPECIFIC_FILES
		actuators.c
		claw.c
		dropper.c
		torpedo.c
	)

	# Append actuators v1 and target all the sources
	list(TRANSFORM ACTUATORS_V1_SPECIFIC_FILES PREPEND ${CMAKE_CURRENT_SOURCE_DIR}/src/actuators_v1/)
	target_sources(actuator_mk2 PUBLIC
		${ACTUATORS_V1_SPECIFIC_FILES}
		src/ros/actuator_v1_parameters.c
	)

	# Add PIO source
	pico_generate_pio_header(actuator_mk2 ${CMAKE_CURRENT_SOURCE_DIR}/src/actuators_v1/torpedo.pio)

	# Link the required libraries for mark 1 actuators
	target_link_libraries(actuator_mk2 PUBLIC
		hardware_adc	# Required for torpedo charge monitoring
		hardware_pio	# Required for torpedo firing
	)
endif()
