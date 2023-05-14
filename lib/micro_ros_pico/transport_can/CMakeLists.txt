add_library(micro_ros_can INTERFACE)
target_sources(micro_ros_can INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}/transport_can.c
)

target_link_libraries(micro_ros_can INTERFACE
    micro_ros_common
    driver_canbus
)

target_include_directories(micro_ros_can INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}/include
)

target_compile_definitions(micro_ros_can INTERFACE MICRO_ROS_TRANSPORT_CAN=1)
