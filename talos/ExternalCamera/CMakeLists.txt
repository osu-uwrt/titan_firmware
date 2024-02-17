cmake_minimum_required(VERSION 3.12)

# Find the titan_firmware dir
execute_process(COMMAND git rev-parse --show-cdup
    WORKING_DIRECTORY "${CMAKE_CURRENT_LIST_DIR}"
    OUTPUT_VARIABLE relative_path_to_repository_root OUTPUT_STRIP_TRAILING_WHITESPACE)
get_filename_component(REPO_DIR "${CMAKE_CURRENT_LIST_DIR}/${relative_path_to_repository_root}" ABSOLUTE)
set(PICO_SDK_PATH ${REPO_DIR}/lib/pico-sdk)

project(dfc_daemon)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

# Add Canmore CPP Core Libraries
add_subdirectory(${REPO_DIR}/tools/lib/canmore_cpp_core canmore_cpp_core)
add_subdirectory(${REPO_DIR}/tools/lib/span_compat span_cpp)

# Find threading support
set(THREADS_PREFER_PTHREAD_FLAG ON)
find_package(Threads REQUIRED)

# Create upload tool
add_executable(dfc_daemon
    main.cpp
    HeartbeatTransmitter.cpp
    CanmoreLinuxServer.cpp
    PTYMaster.cpp
    RemoteTTYServer.cpp
)

target_compile_options(dfc_daemon PUBLIC -Wno-unused-parameter -Wall -Wextra)

target_link_libraries(dfc_daemon
    canmore_cpp_core
    Threads::Threads
)

install(TARGETS dfc_daemon RUNTIME DESTINATION bin)
