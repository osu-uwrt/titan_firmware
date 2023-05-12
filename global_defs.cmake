cmake_minimum_required(VERSION 3.13)

# Set default build type to RelWithDebugInfo
set(CMAKE_BUILD_TYPE RelWithDebInfo CACHE STRING "CMake Build Type")

# Include sdk
set(PICO_SDK_PATH ${REPO_DIR}/lib/pico-sdk)
include(${PICO_SDK_PATH}/external/pico_sdk_import.cmake)

# Add uwrt custom boards (required before SDK init)
include(${REPO_DIR}/lib/uwrt_boards/uwrt_boards.cmake)

# Define global initialization function
function(titan_firmware_init)
    # Make relwithdebuginfo actually like Release
    set(CMAKE_${LANG}_FLAGS_RELWITHDEBINFO "-O3 -DNDEBUG -g")

    # Enable param asserts if in debug mode
    if (CMAKE_BUILD_TYPE STREQUAL "Debug")
        add_compile_definitions(PARAM_ASSERTIONS_ENABLE_ALL=1)
    endif()

    # Enable all warnings
    add_compile_options(-Wall -Wextra)

    # Setup sdk
    set(CMAKE_C_STANDARD 11)
    set(CMAKE_CXX_STANDARD 17)
    pico_sdk_init()

    # Define all custom libraries
    include(${REPO_DIR}/tools/can_bl_tool/enable_upload_command.cmake)
    include(${REPO_DIR}/bootloader/enable_bootloader.cmake)
    add_subdirectory(${REPO_DIR}/lib/ titan_lib)
endfunction()
