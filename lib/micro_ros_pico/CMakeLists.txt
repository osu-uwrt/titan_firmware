set(MICRO_ROS_DIR ${CMAKE_CURRENT_LIST_DIR})

# Prebuilt library location, can be overridden (the nix firmware shell sets MICRO_ROS_LIB_DIR)
if(NOT MICRO_ROS_LIB_DIR)
    if(DEFINED ENV{MICRO_ROS_LIB_DIR})
        set(MICRO_ROS_LIB_DIR $ENV{MICRO_ROS_LIB_DIR})
    else()
        set(MICRO_ROS_LIB_DIR ${MICRO_ROS_DIR}/libmicroros)
    endif()
endif()

if(EXISTS "${MICRO_ROS_LIB_DIR}" AND EXISTS "${MICRO_ROS_LIB_DIR}/libmicroros.a" AND EXISTS "${MICRO_ROS_LIB_DIR}/include")
    add_subdirectory(microros_common)
    add_subdirectory(transport_can)
    add_subdirectory(transport_eth)
    add_subdirectory(transport_usb)
else()
    message(WARNING "Could not locate prebuilt libmicroros install at ${MICRO_ROS_LIB_DIR}!\n"
            "Please install docker and run './build.sh' in the lib/micro_ros_pico directory "
            "or copy the 'lib/micro_ros_pico/libmicroros' directory from another machine.\n"
            "MicroROS library targets will not be created for this build")
endif()
