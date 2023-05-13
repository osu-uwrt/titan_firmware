add_library(micro_ros_custom_allocator INTERFACE)
target_sources(micro_ros_custom_allocator INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}/custom_allocator.c
)

target_link_libraries(micro_ros_custom_allocator INTERFACE
    ${MICRO_ROS_DIR}/libmicroros/libmicroros.a
)

target_include_directories(micro_ros_custom_allocator INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}/include
    ${MICRO_ROS_DIR}/libmicroros/include
)
