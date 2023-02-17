cmake_minimum_required(VERSION 3.12)

add_library(micro_ros_can INTERFACE)
target_sources(micro_ros_can INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}/transport_can.c
)

target_link_libraries(micro_ros_can INTERFACE
    pico_stdlib
    can_mcp251Xfd

    ${MICRO_ROS_DIR}/libmicroros/libmicroros.a
)

target_include_directories(micro_ros_can INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}/include
    ${MICRO_ROS_DIR}/libmicroros/include
)

target_compile_definitions(micro_ros_can INTERFACE SAFETY_ROS_SUPPORT=1)
target_compile_definitions(micro_ros_can INTERFACE MICRO_ROS_TRANSPORT_CAN=1)
