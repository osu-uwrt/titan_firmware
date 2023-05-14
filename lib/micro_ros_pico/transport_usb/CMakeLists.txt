add_library(micro_ros_usb INTERFACE)
target_sources(micro_ros_usb INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}/transport_usb.c
)

target_link_libraries(micro_ros_usb INTERFACE
    micro_ros_common
    titan_dual_stdio_usb
)

target_include_directories(micro_ros_usb INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}/include
)

# Configure sdk parameters
target_compile_definitions(micro_ros_usb INTERFACE PICO_STDIO_USB_ENABLE_UNSENT_BUFFER=1)
target_compile_definitions(micro_ros_usb INTERFACE PICO_STDIO_USB_STDOUT_TIMEOUT_US=10000)
target_compile_definitions(micro_ros_usb INTERFACE MICRO_ROS_TRANSPORT_USB=1)
