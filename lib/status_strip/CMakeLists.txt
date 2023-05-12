cmake_minimum_required(VERSION 3.12)

add_library(status_strip INTERFACE)

target_sources(status_strip INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}/src/status_strip.c
)
# TODO: Make not generate pioasm unless target included
pico_generate_pio_header(status_strip ${CMAKE_CURRENT_LIST_DIR}/src/ws2812.pio)

target_include_directories(status_strip INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}/include
)

target_link_libraries(status_strip INTERFACE
    hardware_pio
	hardware_dma
)
