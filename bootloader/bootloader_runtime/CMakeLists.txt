add_library(bootloader_runtime INTERFACE)

target_sources(bootloader_runtime INTERFACE
	${CMAKE_CURRENT_LIST_DIR}/boot_app.c
	${CMAKE_CURRENT_LIST_DIR}/clocks.c
	${CMAKE_CURRENT_LIST_DIR}/crc32.c
	${CMAKE_CURRENT_LIST_DIR}/crt0.S
	${CMAKE_CURRENT_LIST_DIR}/flash.c
    ${CMAKE_CURRENT_LIST_DIR}/runtime.c
	${CMAKE_CURRENT_LIST_DIR}/timer.c
)

target_include_directories(bootloader_runtime INTERFACE
	${CMAKE_CURRENT_LIST_DIR}/include
	${CMAKE_CURRENT_LIST_DIR}/include_sdk_override
)

# Linking and early initialization
# Required since we want to have a custom memory map and crt0, so we can't use pico_standard_link
# Instead we get to reimplement it all here

# Configure the linker using options we want from pico_standard_link
target_link_options(bootloader_runtime INTERFACE "LINKER:--script=${CMAKE_CURRENT_LIST_DIR}/memmap_bootloader.ld")
target_link_options(bootloader_runtime INTERFACE "LINKER:-z,max-page-size=4096")
target_compile_options(bootloader_runtime INTERFACE -ffunction-sections -fdata-sections)
target_link_options(bootloader_runtime INTERFACE "LINKER:--gc-sections")

# Binary info was removed from the linker to save space (only 16K), so tell the sdk to not use it
target_compile_definitions(bootloader_runtime INTERFACE PICO_NO_BINARY_INFO=1)

# Set the boot stage 2 based on target configuration
target_link_libraries(bootloader_runtime INTERFACE $<IF:$<BOOL:$<TARGET_PROPERTY:PICO_TARGET_BOOT_STAGE2>>,$<TARGET_PROPERTY:PICO_TARGET_BOOT_STAGE2>,bs2_default>_library)

target_link_libraries(bootloader_runtime INTERFACE
	# Minimum headers required for pico sdk project
	pico_base_headers

	# Required for bootrom function lookups
	pico_bootrom

	# Tell compiler to use bootrom functions
	pico_bit_ops
	pico_divider
	pico_mem_ops
	pico_float

	# Simple libraries to make writing the bootloader not painful
	hardware_pll
	hardware_resets
	hardware_watchdog
	hardware_xosc
)