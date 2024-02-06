cmake_minimum_required(VERSION 3.12)

# Include local files
execute_process(COMMAND git rev-parse --show-cdup
    WORKING_DIRECTORY "${CMAKE_CURRENT_LIST_DIR}"
    OUTPUT_VARIABLE relative_path_to_repository_root OUTPUT_STRIP_TRAILING_WHITESPACE)
get_filename_component(REPO_DIR "${CMAKE_CURRENT_LIST_DIR}/${relative_path_to_repository_root}" ABSOLUTE)
set(PICO_SDK_PATH ${REPO_DIR}/lib/pico-sdk)

project(canmore_cli)

set(CMAKE_CXX_STANDARD 17)

# Add common titan libraries
add_subdirectory(../lib titan_tools_lib)

# Create upload tool
add_executable(canmore_cli
    src/main.cpp
    src/discovery.cpp
    src/cli_core.cpp
    src/cli_selection.cpp
    src/application_cli.cpp
    src/bootloader_cli.cpp
    src/linux_cli.cpp
    src/gdbserver.cpp
)

target_include_directories(canmore_cli PUBLIC include/)

target_compile_options(canmore_cli PUBLIC -Wno-psabi -Wall -Wextra)

target_link_libraries(canmore_cli
        canmore_cpp
        rp2040_flash_itf
        binary_info_cpp
)

install(TARGETS canmore_cli RUNTIME DESTINATION bin)
