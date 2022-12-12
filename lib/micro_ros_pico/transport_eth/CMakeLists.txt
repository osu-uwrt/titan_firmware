cmake_minimum_required(VERSION 3.12)

# Setup Wiznet
set(WIZNET_CHIP W5500)

if(NOT DEFINED WIZNET_CHIP)
    message(FATAL_ERROR "WIZNET_CHIP not defined")
elseif(${WIZNET_CHIP} STREQUAL W5100S)
    add_definitions(-D_WIZCHIP_=W5100S)
elseif(${WIZNET_CHIP} STREQUAL W5500)
    add_definitions(-D_WIZCHIP_=W5500)
else()
    message(FATAL_ERROR "WIZNET_CHIP is wrong = ${WIZNET_CHIP}")
endif()

set(WIZNET_DIR ${REPO_DIR}/lib/ioLibrary_Driver)
set(PORT_DIR ${REPO_DIR}/lib/ioLibrary_Port)

#message(FATAL_ERROR ${REPO_DIR})
include(${REPO_DIR}/lib/ioLibrary_import.cmake)
add_subdirectory(${PORT_DIR} ioLibrary_port)

# Setup ethernet
add_library(micro_ros_eth INTERFACE)
target_sources(micro_ros_eth INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}/pico_eth_transport.c
)

target_link_libraries(micro_ros_eth INTERFACE
    pico_stdlib

	ETHERNET_FILES
	IOLIBRARY_FILES

    ${MICRO_ROS_DIR}/libmicroros/libmicroros.a
)

target_include_directories(micro_ros_eth INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}
    ${MICRO_ROS_DIR}/libmicroros/include
)
