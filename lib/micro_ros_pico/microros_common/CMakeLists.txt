add_library(micro_ros_common INTERFACE)
target_sources(micro_ros_common INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}/micro_ros_common.c
)

target_link_libraries(micro_ros_common INTERFACE
    pico_time
    ${MICRO_ROS_DIR}/libmicroros/libmicroros.a
)

target_include_directories(micro_ros_common INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}/include
    ${MICRO_ROS_DIR}/libmicroros/include
)
