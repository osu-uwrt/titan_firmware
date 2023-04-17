cmake_minimum_required(VERSION 3.12)

add_library(dynamixel INTERFACE)

target_sources(dynamixel INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}/src/dynamixel.c
    ${CMAKE_CURRENT_LIST_DIR}/src/dxl_packet.c
    ${CMAKE_CURRENT_LIST_DIR}/src/dynamixel_comms.c
)

target_include_directories(dynamixel INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}/include
)

target_link_libraries(dynamixel INTERFACE
    pico_stdlib
)
