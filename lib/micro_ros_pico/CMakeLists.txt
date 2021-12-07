cmake_minimum_required(VERSION 3.12)

# Requires ${REPO_DIR} be set, but should be set when calling global_defs.cmake anyways

set(MICRO_ROS_PICO_SDK_REPO ${REPO_DIR}/lib/micro_ros_raspberrypi_pico_sdk)

add_library(micro_ros_pico INTERFACE)
target_sources(micro_ros_pico INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}/pico_uart_transport.c
)

target_link_libraries(micro_ros_pico INTERFACE pico_stdlib ${MICRO_ROS_PICO_SDK_REPO}/libmicroros/libmicroros.a)

target_include_directories(micro_ros_pico INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}
    ${MICRO_ROS_PICO_SDK_REPO}/libmicroros/include
)

SET(CMAKE_C_FLAGS  "${CMAKE_C_FLAGS} -ffunction-sections -fdata-sections")
SET(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -ffunction-sections -fdata-sections")

# Configure sdk parameters
uwrt_enable_dual_uart(micro_ros_pico INTERFACE)
target_compile_definitions(micro_ros_pico INTERFACE PICO_STDIO_USB_CONNECT_WAIT_TIMEOUT_MS=2000)