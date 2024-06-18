
cmake_minimum_required(VERSION 3.12)

# Find threading support
set(THREADS_PREFER_PTHREAD_FLAG ON)
find_package(Threads REQUIRED)

set(NO_AMENT 1)
add_subdirectory(${REPO_DIR}/lib/titan_canmore titan_canmore)

# Define sources
add_library(canmore_dbg_cpp INTERFACE)

target_sources(canmore_dbg_cpp INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}/src/Discovery.cpp
    ${CMAKE_CURRENT_LIST_DIR}/src/DebugClient.cpp
    ${CMAKE_CURRENT_LIST_DIR}/src/CANDiscovery.cpp
    ${CMAKE_CURRENT_LIST_DIR}/src/EthernetDiscovery.cpp
    ${CMAKE_CURRENT_LIST_DIR}/src/BootloaderClient.cpp
    ${CMAKE_CURRENT_LIST_DIR}/src/RemoteTTYClient.cpp
    ${CMAKE_CURRENT_LIST_DIR}/src/LinuxClient.cpp
    ${CMAKE_CURRENT_LIST_DIR}/src/ImageReceiver.cpp
)

target_include_directories(canmore_dbg_cpp INTERFACE ${CMAKE_CURRENT_LIST_DIR}/include)

target_link_libraries(canmore_dbg_cpp INTERFACE
    canmore_cpp
    Threads::Threads

    # Tool Libraries
    rp2040_flash_itf
)
