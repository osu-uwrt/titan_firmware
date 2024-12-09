add_library(driver_status_strip INTERFACE)

target_sources(driver_status_strip INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}/status_strip.c
)
# TODO: Make not generate pioasm unless target included
pico_generate_pio_header(driver_status_strip ${CMAKE_CURRENT_LIST_DIR}/ws2812.pio)

target_include_directories(driver_status_strip INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}/include
)

target_link_libraries(driver_status_strip INTERFACE
    hardware_pio
    hardware_dma
)
