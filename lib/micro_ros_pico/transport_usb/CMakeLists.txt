cmake_minimum_required(VERSION 3.12)

add_library(micro_ros_usb INTERFACE)
target_sources(micro_ros_usb INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}/transport_usb.c
)

target_link_libraries(micro_ros_usb INTERFACE
    pico_stdlib
    dual_serial_stdio_usb
    micro_ros_custom_allocator
    ${MICRO_ROS_DIR}/libmicroros/libmicroros.a
)

target_include_directories(micro_ros_usb INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}/include
    ${MICRO_ROS_DIR}/libmicroros/include
)

target_compile_definitions(micro_ros_usb INTERFACE SAFETY_ROS_SUPPORT=1)

# Configure sdk parameters
target_compile_definitions(micro_ros_usb INTERFACE PICO_STDIO_USB_ENABLE_UNSENT_BUFFER=1)
target_compile_definitions(micro_ros_usb INTERFACE PICO_STDIO_USB_STDOUT_TIMEOUT_US=10000)
target_compile_definitions(micro_ros_usb INTERFACE MICRO_ROS_TRANSPORT_USB=1)
