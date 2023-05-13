cmake_minimum_required(VERSION 3.12)

add_library(sht41 INTERFACE)

target_sources(sht41 INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}/src/sht41.c
)

target_link_libraries(sht41 INTERFACE pico_stdlib async_i2c)

target_include_directories(sht41 INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}/include
)
