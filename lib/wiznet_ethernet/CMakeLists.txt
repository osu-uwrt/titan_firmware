cmake_minimum_required(VERSION 3.12)

add_library(wiznet_ethernet INTERFACE)

target_sources(wiznet_ethernet INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}/src/base_networking.c
    ${CMAKE_CURRENT_LIST_DIR}/src/udp_networking.c
    ${CMAKE_CURRENT_LIST_DIR}/src/w5100.c
)

target_link_libraries(wiznet_ethernet INTERFACE
    hardware_gpio
    hardware_regs
    hardware_spi
)

target_include_directories(wiznet_ethernet INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}/include
)
