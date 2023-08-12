cmake_minimum_required(VERSION 3.12)

# Add required libraries
# Note PICO_SDK_PATH must be set
add_subdirectory(${PICO_SDK_PATH}/src/host/pico_platform pico_platform)
add_subdirectory(${PICO_SDK_PATH}/src/common/boot_picoboot boot_picoboot_headers)
add_subdirectory(${PICO_SDK_PATH}/src/common/pico_usb_reset_interface pico_usb_reset_interface)
add_subdirectory(picoboot_connection/)

list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_LIST_DIR}/cmake)

find_package(LIBUSB)
if (NOT LIBUSB_FOUND)
    message(FATAL_ERROR "Cannot build with pico_usb support - LibUSB not found")
endif()

add_library(pico_usb INTERFACE)

target_sources(pico_usb INTERFACE
        ${CMAKE_CURRENT_LIST_DIR}/src/OpenOCDInstance.cpp
	${CMAKE_CURRENT_LIST_DIR}/src/RP2040OCDTarget.cpp
        ${CMAKE_CURRENT_LIST_DIR}/src/PicoprobeClient.cpp
	${CMAKE_CURRENT_LIST_DIR}/src/USBBootrom.cpp
	${CMAKE_CURRENT_LIST_DIR}/src/USBDevice.cpp
	${CMAKE_CURRENT_LIST_DIR}/src/USBDiscovery.cpp
	${CMAKE_CURRENT_LIST_DIR}/flash_getid_stub/flash_getid_compiled.c
)

target_include_directories(pico_usb INTERFACE ${CMAKE_CURRENT_LIST_DIR}/include ${LIBUSB_INCLUDE_DIR})
target_include_directories(pico_usb INTERFACE ${CMAKE_CURRENT_LIST_DIR}/flash_getid_stub/include)

# Include hardware registers, can't do cmake include since it pulls in a lot more than we need
target_include_directories(pico_usb INTERFACE ${PICO_SDK_PATH}/src/rp2040/hardware_regs/include)

target_link_libraries(pico_usb INTERFACE
	boot_picoboot_headers
	pico_platform_headers
	pico_usb_reset_interface_headers
	${LIBUSB_LIBRARIES}

	# Tool Libraries
	rp2040_flash_itf
	binary_info_cpp

	# Local folder libraries
	picoboot_connection
	picoboot_connection_cxx
)
