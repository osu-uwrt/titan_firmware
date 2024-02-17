cmake_minimum_required(VERSION 3.12)

# Include required libraries
add_subdirectory(${REPO_DIR}/lib/titan_canmore titan_canmore)

# Define sources
add_library(canmore_cpp_core INTERFACE)

target_sources(canmore_cpp_core INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}/src/RegMappedClient.cpp
    ${CMAKE_CURRENT_LIST_DIR}/src/RegMappedCANClient.cpp
    ${CMAKE_CURRENT_LIST_DIR}/src/RegMappedEthernetClient.cpp
    ${CMAKE_CURRENT_LIST_DIR}/src/RegMappedServer.cpp
    ${CMAKE_CURRENT_LIST_DIR}/src/RemoteTTYStream.cpp
    ${CMAKE_CURRENT_LIST_DIR}/src/CANSocket.cpp
)

target_include_directories(canmore_cpp_core INTERFACE ${CMAKE_CURRENT_LIST_DIR}/include)

target_link_libraries(canmore_cpp_core INTERFACE
    titan_canmore
    span_cpp
)
