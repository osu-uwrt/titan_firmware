cmake_minimum_required(VERSION 3.12)

add_library(async_i2c INTERFACE)

target_sources(async_i2c INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}/src/async_i2c.c
)

target_include_directories(async_i2c INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}/include
)

target_link_libraries(async_i2c INTERFACE
    pico_stdlib
    hardware_i2c
)
