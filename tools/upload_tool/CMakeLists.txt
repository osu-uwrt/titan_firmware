cmake_minimum_required(VERSION 3.12)

# Include local files
execute_process(COMMAND git rev-parse --show-cdup
	WORKING_DIRECTORY "${CMAKE_CURRENT_LIST_DIR}"
	OUTPUT_VARIABLE relative_path_to_repository_root OUTPUT_STRIP_TRAILING_WHITESPACE)
get_filename_component(REPO_DIR "${CMAKE_CURRENT_LIST_DIR}/${relative_path_to_repository_root}" ABSOLUTE)
set(PICO_SDK_PATH ${REPO_DIR}/lib/pico-sdk)

project(upload_tool)

set(CMAKE_CXX_STANDARD 17)

list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_LIST_DIR}/cmake)

add_subdirectory(${PICO_SDK_PATH}/src/common/pico_binary_info pico_binary_info)
add_subdirectory(${PICO_SDK_PATH}/src/common/boot_uf2 boot_uf2_headers)
add_subdirectory(${PICO_SDK_PATH}/src/common/boot_picoboot boot_picoboot_headers)
add_subdirectory(${PICO_SDK_PATH}/src/common/pico_usb_reset_interface pico_usb_reset_interface)
add_subdirectory(${PICO_SDK_PATH}/src/host/pico_platform pico_platform)
add_subdirectory(${REPO_DIR}/lib/titan_binary_info titan_binary_info)
add_subdirectory(${REPO_DIR}/lib/titan_canmore titan_canmore)
add_subdirectory(picoboot_connection/)

add_executable(upload_tool
	main.cpp
	BinaryInfo.cpp
	DeviceMap.cpp
	UF2.cpp
	FlashUI.cpp
	canmore_cpp/Discovery.cpp
	canmore_cpp/DebugClient.cpp
	canmore_cpp/CANDiscovery.cpp
	canmore_cpp/EthernetDiscovery.cpp
	canmore_cpp/RegMappedClient.cpp
	canmore_cpp/RegMappedCANClient.cpp
	canmore_cpp/RegMappedEthernetClient.cpp
	canmore_cpp/BootloaderClient.cpp
        pico_usb/OpenOCDInstance.cpp
	pico_usb/RP2040OCDTarget.cpp
        pico_usb/PicoprobeClient.cpp
	pico_usb/USBBootrom.cpp
	pico_usb/USBDevice.cpp
	pico_usb/USBDiscovery.cpp
)

# Copy in DeviceList
set(DEVICE_DEFINITIONS_FILE DeviceList.jsonc)
configure_file(${DEVICE_DEFINITIONS_FILE} ${DEVICE_DEFINITIONS_FILE} COPYONLY)
target_compile_definitions(upload_tool PUBLIC DEVICE_DEFINITIONS_FILE="${DEVICE_DEFINITIONS_FILE}")

target_compile_options(upload_tool PUBLIC -Wno-psabi -Wall -Wextra)
# Include hardware registers, can't do cmake include since it pulls in a lot more than we need
target_include_directories(upload_tool PUBLIC ${PICO_SDK_PATH}/src/rp2040/hardware_regs/include)

target_link_libraries(upload_tool
	boot_uf2_headers
	pico_binary_info
	titan_binary_info
	titan_canmore

	boot_picoboot_headers
	pico_platform_headers
	pico_usb_reset_interface_headers
	picoboot_connection
	picoboot_connection_cxx
	${LIBUSB_LIBRARIES}

	pthread
	usb-1.0
)
