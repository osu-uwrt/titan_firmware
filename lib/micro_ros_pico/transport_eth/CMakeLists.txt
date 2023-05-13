cmake_minimum_required(VERSION 3.12)

# Setup ethernet
add_library(micro_ros_eth INTERFACE)
target_sources(micro_ros_eth INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}/transport_eth.c
)

target_link_libraries(micro_ros_eth INTERFACE
    pico_time
    pico_unique_id
    wiznet_ethernet

    canmore_protocol
    canmore_debug_interface
    titan_binary_info

    ${MICRO_ROS_DIR}/libmicroros/libmicroros.a
)

target_include_directories(micro_ros_eth INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}/include
    ${MICRO_ROS_DIR}/libmicroros/include
)

target_compile_definitions(micro_ros_eth INTERFACE SAFETY_ROS_SUPPORT=1)
target_compile_definitions(micro_ros_eth INTERFACE MICRO_ROS_TRANSPORT_ETH=1)
