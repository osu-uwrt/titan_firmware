cmake_minimum_required(VERSION 3.13)

# Include the runtime subdirectory
add_subdirectory(bootloader_runtime)

# Define common bootloader base
add_library(bootloader_common INTERFACE)
target_sources(bootloader_common INTERFACE
	main.c
	bl_server.c
)
target_compile_options(bootloader_common INTERFACE -Wall -Wextra -Wno-format)
target_include_directories(bootloader_common INTERFACE include)

# Define linking and targets
target_link_libraries(bootloader_common INTERFACE
	bootloader_runtime
	hardware_uart
)

# Define CAN bus bootloader variant
add_executable(bootloader_can
	can_bl_interface.c
)

# Set version: major minor release_type (see build_version.h for more info)
# Increment this if any compatability changes occur
# Release Types: PROTO, DEV, STABLE
# Note: The bootloader verison is tied to the protocol, changing this will alert the client of a protocol change
generate_version_tag(bootloader_can 1 0 PROTO)

target_link_libraries(bootloader_can
	bootloader_common

	hardware_spi
	can_mcp251Xfd_ll_driver
	canmore_protocol
)
pico_add_extra_outputs(bootloader_can)

