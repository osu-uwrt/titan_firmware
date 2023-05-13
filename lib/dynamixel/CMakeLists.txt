cmake_minimum_required(VERSION 3.12)

add_library(dynamixel INTERFACE)

target_sources(dynamixel INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}/src/async_uart.c
    ${CMAKE_CURRENT_LIST_DIR}/src/dynamixel.c
    ${CMAKE_CURRENT_LIST_DIR}/src/dxl_packet.c
    ${CMAKE_CURRENT_LIST_DIR}/src/dynamixel_comms.c
    ${CMAKE_CURRENT_LIST_DIR}/src/dynamixel_reg.c
    ${CMAKE_CURRENT_LIST_DIR}/src/dynamixel_schedule.c
)
pico_generate_pio_header(dynamixel ${CMAKE_CURRENT_LIST_DIR}/src/uart_multidrop.pio)

target_include_directories(dynamixel INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}/include
)

target_link_libraries(dynamixel INTERFACE
    hardware_pio
    hardware_dma
    basic_queue
)
