set(MICRO_ROS_DIR ${CMAKE_CURRENT_LIST_DIR})

add_subdirectory(transport_can)
add_subdirectory(transport_eth)
add_subdirectory(transport_usb)

SET(CMAKE_C_FLAGS  "${CMAKE_C_FLAGS} -ffunction-sections -fdata-sections")
SET(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -ffunction-sections -fdata-sections")