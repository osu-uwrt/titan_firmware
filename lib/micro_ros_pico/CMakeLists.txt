cmake_minimum_required(VERSION 3.12)

add_library(micro_ros_pico INTERFACE)
target_sources(micro_ros_pico INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}/pico_uart_transport.c
)

target_link_libraries(micro_ros_pico INTERFACE pico_stdlib ${CMAKE_CURRENT_LIST_DIR}/libmicroros/libmicroros.a)

target_include_directories(micro_ros_pico INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}
    ${CMAKE_CURRENT_LIST_DIR}/libmicroros/include
)

SET(CMAKE_C_FLAGS  "${CMAKE_C_FLAGS} -ffunction-sections -fdata-sections")
SET(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -ffunction-sections -fdata-sections")

# Configure sdk parameters
uwrt_enable_dual_uart(micro_ros_pico INTERFACE)
target_compile_definitions(micro_ros_pico INTERFACE PICO_STDIO_USB_ENABLE_UNSENT_BUFFER=1)
target_compile_definitions(micro_ros_pico INTERFACE PICO_STDIO_USB_STDOUT_TIMEOUT_US=10000)