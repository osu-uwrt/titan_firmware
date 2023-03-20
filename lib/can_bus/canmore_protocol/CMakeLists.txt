cmake_minimum_required(VERSION 3.12)

add_library(canmore_protocol INTERFACE)

target_sources(canmore_protocol INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}/src/msg_encoding.c
    ${CMAKE_CURRENT_LIST_DIR}/src/reg_mapped_server.c
)

target_include_directories(canmore_protocol INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}/include
)
