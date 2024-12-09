add_library(micro_ros_eth INTERFACE)
target_sources(micro_ros_eth INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}/transport_eth.c
)

target_link_libraries(micro_ros_eth INTERFACE
    micro_ros_common

    driver_wiznet
    pico_unique_id
    titan_binary_info
    titan_debug
    titan_logger
)

target_include_directories(micro_ros_eth INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}/include
)

target_compile_definitions(micro_ros_eth INTERFACE MICRO_ROS_TRANSPORT_ETH=1)
