cmake_minimum_required(VERSION 3.12)

add_library(basic_logger INTERFACE)

target_sources(basic_logger INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}/basic_logger.c
)

target_link_libraries(basic_logger INTERFACE pico_time)

target_include_directories(basic_logger INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}/include
)
