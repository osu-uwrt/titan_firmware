set(MICRO_ROS_DIR ${CMAKE_CURRENT_LIST_DIR})

if(EXISTS "${MICRO_ROS_DIR}/libmicroros" AND EXISTS "${MICRO_ROS_DIR}/libmicroros/libmicroros.a" AND EXISTS "${MICRO_ROS_DIR}/libmicroros/include")
    add_subdirectory(microros_common)
    add_subdirectory(transport_can)
    add_subdirectory(transport_eth)
    add_subdirectory(transport_usb)
else()
    message(WARNING "Could not locate prebuilt libmicroros install!\n"
            "Please install docker and run './build.sh' in the lib/micro_ros_pico directory "
            "or copy the 'lib/micro_ros_pico/libmicroros' directory from another machine.\n"
            "MicroROS library targets will not be created for this build")
endif()
