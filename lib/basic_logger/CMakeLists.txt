cmake_minimum_required(VERSION 3.12)

add_library(basic_logging INTERFACE)

target_sources(basic_logging INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}/basic_logging.c
)

target_link_libraries(basic_logging INTERFACE pico_stdlib)

target_include_directories(basic_logging INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}/include
)
