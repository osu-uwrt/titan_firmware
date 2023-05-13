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
    hardware_clocks
    hardware_irq
    hardware_gpio
    hardware_spi
    hardware_sync
    pico_multicore
    pico_time

    basic_queue
    canmore_protocol
)
