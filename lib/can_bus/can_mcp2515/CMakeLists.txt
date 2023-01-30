cmake_minimum_required(VERSION 3.12)

add_library(can_mcp2515 INTERFACE)

target_sources(can_mcp2515 INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}/src/can_core1.cpp
    ${CMAKE_CURRENT_LIST_DIR}/src/can_interface.c
    ${CMAKE_CURRENT_LIST_DIR}/src/mcp2515/mcp2515.cpp
)

target_include_directories(can_mcp2515 INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}/include
)

target_link_libraries(can_mcp2515 INTERFACE
    pico_stdlib
    pico_multicore
    hardware_spi

    basic_queue
    canmore_protocol
)
