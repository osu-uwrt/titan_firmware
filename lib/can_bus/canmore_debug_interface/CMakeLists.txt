cmake_minimum_required(VERSION 3.12)

add_library(canmore_debug_interface INTERFACE)

target_sources(canmore_debug_interface INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}/src/canmore_debug.c
)

target_include_directories(canmore_debug_interface INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}/include
)

target_link_libraries(canmore_debug_interface INTERFACE
    canmore_protocol
)
