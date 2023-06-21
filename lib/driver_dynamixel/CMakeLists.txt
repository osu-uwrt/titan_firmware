add_library(driver_dynamixel INTERFACE)

target_sources(driver_dynamixel INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}/src/async_uart.c
    ${CMAKE_CURRENT_LIST_DIR}/src/dynamixel.c
    ${CMAKE_CURRENT_LIST_DIR}/src/dxl_packet.c
    ${CMAKE_CURRENT_LIST_DIR}/src/dynamixel_comms.c
    ${CMAKE_CURRENT_LIST_DIR}/src/dynamixel_reg.c
    ${CMAKE_CURRENT_LIST_DIR}/src/dynamixel_schedule.c
)
pico_generate_pio_header(driver_dynamixel ${CMAKE_CURRENT_LIST_DIR}/src/uart_multidrop.pio)

target_include_directories(driver_dynamixel INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}/include
)

target_link_libraries(driver_dynamixel INTERFACE
    hardware_dma
    hardware_pio
    titan_logger
    titan_queue
)
