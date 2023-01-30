cmake_minimum_required(VERSION 3.12)

add_library(can_pio INTERFACE)

target_sources(can_pio INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}/src/can_core1.c
    ${CMAKE_CURRENT_LIST_DIR}/src/can_interface.c
    ${CMAKE_CURRENT_LIST_DIR}/src/can2040.c
)

target_include_directories(can_pio INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}/include
)

target_link_libraries(can_pio INTERFACE
    pico_stdlib
    pico_multicore

    basic_queue
    canmore_protocol
)
