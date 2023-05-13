cmake_minimum_required(VERSION 3.12)

# Low level driver (used when only the mcp251Xfd internal driver is needed, not the wrapper)
add_library(can_mcp251Xfd_ll_driver INTERFACE)

target_sources(can_mcp251Xfd_ll_driver INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}/src/mcp251Xfd/MCP251XFD.c
    ${CMAKE_CURRENT_LIST_DIR}/src/mcp251Xfd/CRC16_CMS.c
)

target_include_directories(can_mcp251Xfd_ll_driver INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}/src/mcp251Xfd
)


add_library(can_mcp251Xfd INTERFACE)

target_sources(can_mcp251Xfd INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}/src/can_interface.c
    ${CMAKE_CURRENT_LIST_DIR}/src/can_mcp251XFD.c
    ${CMAKE_CURRENT_LIST_DIR}/src/mcp251Xfd/MCP251XFD.c
    ${CMAKE_CURRENT_LIST_DIR}/src/mcp251Xfd/CRC16_CMS.c
)

target_include_directories(can_mcp251Xfd INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}/include
)

target_link_libraries(can_mcp251Xfd INTERFACE
    pico_unique_id
    hardware_clocks
    hardware_gpio
    hardware_irq
    hardware_spi
    pico_sync

    basic_queue
    canmore_protocol
    canmore_debug_interface
    titan_binary_info   # For client id reporting
)
