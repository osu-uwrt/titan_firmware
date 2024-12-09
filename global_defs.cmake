cmake_minimum_required(VERSION 3.13)

# Set default build type to RelWithDebugInfo
set(CMAKE_BUILD_TYPE RelWithDebInfo CACHE STRING "CMake Build Type")

# Include sdk
set(PICO_SDK_PATH ${REPO_DIR}/lib/pico-sdk)
if (EXISTS ${PICO_SDK_PATH}/external/pico_sdk_import.cmake)
    include(${PICO_SDK_PATH}/external/pico_sdk_import.cmake)
else()
    message(FATAL_ERROR "Could not locate pico-sdk import script. Ensure all submodules are pulled.")
endif()

# Define global initialization function
function(titan_firmware_init)
    # Make relwithdebuginfo actually like Release (-O3 instead of -O2), but with enhanced debug info (-ggdb3)
    set(CMAKE_ASM_FLAGS_RELWITHDEBINFO "-O3 -DNDEBUG -ggdb3" PARENT_SCOPE)
    set(CMAKE_C_FLAGS_RELWITHDEBINFO "-O3 -DNDEBUG -ggdb3" PARENT_SCOPE)
    set(CMAKE_CXX_FLAGS_RELWITHDEBINFO "-O3 -DNDEBUG -ggdb3" PARENT_SCOPE)

    # Enable more debug info so we get fancy macro expansion during debug (normally just -g)
    set(CMAKE_ASM_FLAGS_DEBUG "-Og -ggdb3" PARENT_SCOPE)
    set(CMAKE_C_FLAGS_DEBUG "-Og -ggdb3" PARENT_SCOPE)
    set(CMAKE_CXX_FLAGS_DEBUG "-Og -ggdb3" PARENT_SCOPE)

    # Split function and data into sections to allow linker to optimize unused functions
    add_compile_options(-ffunction-sections -fdata-sections)

    # Map all __FILE__ invoctaions to remove the prefix before the repo root - Makes logging cleaner
    # Only supported by GCC version 8.0 and above
    if(CMAKE_COMPILER_IS_GNUCC AND CMAKE_C_COMPILER_VERSION VERSION_GREATER 8.0)
        add_compile_options(-fmacro-prefix-map=${REPO_DIR}/=)
    endif()

    # Enable param asserts if in debug mode
    if (CMAKE_BUILD_TYPE STREQUAL "Debug")
        add_compile_definitions(PARAM_ASSERTIONS_ENABLE_ALL=1)
    endif()

    # Enable all warnings
    add_compile_options(-Wall -Wextra)

    # Load custom board definitions (must occur before SDK init)
    include(${REPO_DIR}/lib/titan_boards/titan_boards.cmake)

    # Setup sdk
    set(CMAKE_C_STANDARD 11 PARENT_SCOPE)
    set(CMAKE_CXX_STANDARD 17 PARENT_SCOPE)
    pico_sdk_init()

    # Add support for upload command
    include(${REPO_DIR}/tools/upload_tool/enable_upload_command.cmake)

    # Import all libraries
    add_subdirectory(${REPO_DIR}/lib/ titan_lib)
endfunction()
