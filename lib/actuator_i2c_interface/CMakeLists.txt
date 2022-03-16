cmake_minimum_required(VERSION 3.12)

add_library(actuator_i2c_interface INTERFACE)

target_sources(actuator_i2c_interface INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}/crc8_calc.c
)

target_link_libraries(actuator_i2c_interface INTERFACE pico_stdlib)

target_include_directories(actuator_i2c_interface INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}/include
)
