cmake_minimum_required(VERSION 3.13)

if (NOT DEFINED BOOTLOADER_INTERFACE)
	message(FATAL_ERROR "The bootloader must be configured with the appropriate BOOTLOADER_INTERFACE")
endif()

# Include the runtime subdirectory
add_subdirectory(titan_binary_info)
add_subdirectory(bootloader_runtime)

# Define common bootloader base
add_executable(titan_bootloader
	main.c
	bl_server.c
)
target_compile_options(titan_bootloader PUBLIC -Wall -Wextra -Wno-format)
target_include_directories(titan_bootloader PUBLIC include)

# Target minimum compile size to fit in 16K
target_compile_options(titan_bootloader PUBLIC -Os)

# Define linking and targets
target_link_libraries(titan_bootloader PUBLIC
	bootloader_runtime
	hardware_uart

	# We need canmore protocol in all builds as the bl_server depends on it
	canmore_protocol
)

# Set version: major minor release_type (see build_version.h for more info)
# Increment this if any compatability changes occur
# Release Types: PROTO, DEV, STABLE
# Note: The bootloader verison is tied to the protocol, changing this will alert the client of a protocol change
generate_version_tag(titan_bootloader 1 0 STABLE)

pico_add_extra_outputs(titan_bootloader)

####################################
# Define Bootloader Variants
####################################

if (BOOTLOADER_INTERFACE MATCHES "can")
	# CAN Bus Bootloader Variant
	target_sources(titan_bootloader PUBLIC
		can_bl_interface.c
	)

	target_link_libraries(titan_bootloader PUBLIC
		hardware_spi
		can_mcp251Xfd_ll_driver
	)

elseif (BOOTLOADER_INTERFACE MATCHES "eth")
	# Ethernet bootloader variant
	target_sources(titan_bootloader PUBLIC
		eth_bl_interface.c
	)

	target_link_libraries(titan_bootloader PUBLIC
		wiznet_ethernet
	)

else()
	message(FATAL_ERROR "Invalid BOOTLOADER_INTERFACE specified: " ${BOOTLOADER_INTERFACE})
endif()