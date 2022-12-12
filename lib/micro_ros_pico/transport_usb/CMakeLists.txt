cmake_minimum_required(VERSION 3.12)

add_library(micro_ros_usb INTERFACE)
target_sources(micro_ros_usb INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}/pico_uart_transport.c
)

target_link_libraries(micro_ros_usb INTERFACE pico_stdlib ${CMAKE_CURRENT_LIST_DIR}/libmicroros/libmicroros.a)

target_include_directories(micro_ros_usb INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}
    ${CMAKE_CURRENT_LIST_DIR}/libmicroros/include
)

target_compile_definitions(micro_ros_usb INTERFACE SAFETY_ROS_SUPPORT=1)

# Configure sdk parameters
uwrt_enable_dual_uart(micro_ros_usb INTERFACE)
target_compile_definitions(micro_ros_usb INTERFACE PICO_STDIO_USB_ENABLE_UNSENT_BUFFER=1)
target_compile_definitions(micro_ros_usb INTERFACE PICO_STDIO_USB_STDOUT_TIMEOUT_US=10000)
