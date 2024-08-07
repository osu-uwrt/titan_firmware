cmake_minimum_required(VERSION 3.13)

# Include global definitions
execute_process(COMMAND git rev-parse --show-cdup
    WORKING_DIRECTORY "${CMAKE_CURRENT_LIST_DIR}"
    OUTPUT_VARIABLE relative_path_to_repository_root OUTPUT_STRIP_TRAILING_WHITESPACE ERROR_STRIP_TRAILING_WHITESPACE)
get_filename_component(REPO_DIR "${CMAKE_CURRENT_LIST_DIR}/${relative_path_to_repository_root}" ABSOLUTE)
include(${REPO_DIR}/global_defs.cmake)

if (NOT DEFINED BOOTLOADER_INTERFACE)
    message(FATAL_ERROR "The bootloader must be configured with the appropriate BOOTLOADER_INTERFACE")
endif()

project(titan_bootloader C CXX ASM)
titan_firmware_init()

# Include the runtime subdirectory
add_subdirectory(bootloader_runtime)

# Define common bootloader base
add_executable(titan_bootloader
    src/main.c
    src/bl_server.c
    src/dbg_uart.c
    src/status_led.c
)
target_include_directories(titan_bootloader PUBLIC include)

# Target minimum compile size to fit in 16K
target_compile_options(titan_bootloader PUBLIC -Os)
target_compile_options(canmore PUBLIC -Os)  # Needs to also be marked as -Os since it's a static library

# Definitions to optimize the binary size
# Disable arg in canmore reg mapped to save a decent amount of space
target_compile_definitions(canmore PUBLIC CANMORE_CONFIG_DISABLE_REG_MAPPED_ARG=1)
# Disable CRC16 table, it's slower but saves 512 bytes
target_compile_definitions(titan_bootloader PUBLIC CRC16CMS_NOTABLE=1)
# Disable bitrate statistics, saves >512 bytes and we don't use it (but will be linked in because it depends on a param)
target_compile_definitions(titan_bootloader PUBLIC MCP251X_DISABLE_BITRATE_STATS=1)

# Define linking and targets
target_link_libraries(titan_bootloader PUBLIC
    bootloader_runtime

    # We need canmore protocol in all builds as the bl_server depends on it
    canmore
)

# Set version: major minor (see titan/version.h for more info)
# Increment this if any compatability changes occur
# Note: The bootloader verison is tied to the protocol, changing this will alert the client of a protocol change
titan_generate_version_tag(titan_bootloader 1 0)

pico_add_extra_outputs(titan_bootloader)

# Disable PIO assembler for bootloader (we don't use it)
if (TARGET PioasmBuild)
    set_target_properties(PioasmBuild PROPERTIES EXCLUDE_FROM_ALL 1 EXCLUDE_FROM_DEFAULT_BUILD 1)
endif()

####################################
# Define Bootloader Variants
####################################

if (BOOTLOADER_INTERFACE MATCHES "can")
    # CAN Bus Bootloader Variant
    target_sources(titan_bootloader PUBLIC
        src/can_bl_interface.c
    )

    target_link_libraries(titan_bootloader PUBLIC
        hardware_spi
        driver_mcp251Xfd_ll_driver
    )

    target_compile_definitions(titan_bootloader PUBLIC TITAN_BOOTLOADER_CAN_INTERFACE=1)

    # We can't take advantage of multiword for standard CAN, disable it to save space
    target_compile_definitions(canmore PUBLIC CANMORE_CONFIG_DISABLE_MULTIWORD=1)

elseif (BOOTLOADER_INTERFACE MATCHES "eth")
    # Ethernet bootloader variant
    target_sources(titan_bootloader PUBLIC
        src/eth_bl_interface.c
    )

    target_compile_definitions(titan_bootloader PUBLIC TITAN_BOOTLOADER_ETH_INTERFACE=1)

    target_link_libraries(titan_bootloader PUBLIC
        driver_wiznet
    )

else()
    message(FATAL_ERROR "Invalid BOOTLOADER_INTERFACE specified: " ${BOOTLOADER_INTERFACE})
endif()
