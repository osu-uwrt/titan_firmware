cmake_minimum_required(VERSION 3.12)

add_library(titan_canmore INTERFACE)

target_sources(titan_canmore INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}/src/msg_encoding.c
    ${CMAKE_CURRENT_LIST_DIR}/src/reg_mapped_server.c
    ${CMAKE_CURRENT_LIST_DIR}/src/reg_mapped_client.c
)

target_include_directories(titan_canmore INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}/include
)
