cmake_minimum_required(VERSION 3.12)

add_library(micro_ros_can INTERFACE)
target_sources(micro_ros_can INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}/pico_can_transport.c
)

target_link_libraries(micro_ros_can INTERFACE
    pico_stdlib
    can_pio

    ${MICRO_ROS_DIR}/libmicroros/libmicroros.a
)

target_include_directories(micro_ros_can INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}
    ${MICRO_ROS_DIR}/libmicroros/include
)
