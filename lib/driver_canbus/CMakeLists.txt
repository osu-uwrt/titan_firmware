cmake_minimum_required(VERSION 3.12)

# Low level driver (used when only the mcp251Xfd internal driver is needed, not the wrapper)
add_library(driver_mcp251Xfd_ll_driver INTERFACE)

target_sources(driver_mcp251Xfd_ll_driver INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}/src/mcp251Xfd/MCP251XFD.c
    ${CMAKE_CURRENT_LIST_DIR}/src/mcp251Xfd/CRC16_CMS.c
)

target_include_directories(driver_mcp251Xfd_ll_driver INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}/src/mcp251Xfd
)


add_library(driver_canbus INTERFACE)

target_sources(driver_canbus INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}/src/can_interface.c
    ${CMAKE_CURRENT_LIST_DIR}/src/can_mcp251XFD.c
    ${CMAKE_CURRENT_LIST_DIR}/src/mcp251Xfd/MCP251XFD.c
    ${CMAKE_CURRENT_LIST_DIR}/src/mcp251Xfd/CRC16_CMS.c
)

target_include_directories(driver_canbus INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}/include
)

target_link_libraries(driver_canbus INTERFACE
    pico_unique_id
    hardware_clocks
    hardware_gpio
    hardware_irq
    hardware_spi
    pico_sync

    titan_canmore
    titan_debug
    titan_binary_info   # For client id reporting
)
