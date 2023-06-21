add_library(driver_wiznet INTERFACE)

target_sources(driver_wiznet INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}/src/base_networking.c
    ${CMAKE_CURRENT_LIST_DIR}/src/udp_networking.c
    ${CMAKE_CURRENT_LIST_DIR}/src/w5100.c
)

target_link_libraries(driver_wiznet INTERFACE
    hardware_regs
    hardware_spi
)

target_include_directories(driver_wiznet INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}/include
)
