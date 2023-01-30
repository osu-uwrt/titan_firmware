cmake_minimum_required(VERSION 3.12)

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
    pico_stdlib
    hardware_spi

    basic_queue
    canmore_protocol
)
