cmake_minimum_required(VERSION 3.12)

add_library(driver_async_i2c INTERFACE)

target_sources(driver_async_i2c INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}/async_i2c.c
)

target_include_directories(driver_async_i2c INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}/include
)

target_link_libraries(driver_async_i2c INTERFACE
    hardware_gpio
    hardware_i2c
    hardware_irq
    hardware_sync
    pico_time
)
