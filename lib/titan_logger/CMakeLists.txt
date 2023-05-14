cmake_minimum_required(VERSION 3.12)

add_library(titan_logger INTERFACE)

target_sources(titan_logger INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}/logger.c
)

target_link_libraries(titan_logger INTERFACE pico_time)

target_include_directories(titan_logger INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}/include
)
