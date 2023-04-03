cmake_minimum_required(VERSION 3.12)

# Include local files
execute_process(COMMAND git rev-parse --show-cdup
	WORKING_DIRECTORY "${CMAKE_CURRENT_LIST_DIR}"
	OUTPUT_VARIABLE relative_path_to_repository_root OUTPUT_STRIP_TRAILING_WHITESPACE)
get_filename_component(REPO_DIR "${CMAKE_CURRENT_LIST_DIR}/${relative_path_to_repository_root}" ABSOLUTE)
set(PICO_SDK_PATH ${REPO_DIR}/lib/pico-sdk)

project(caN_bl_tool)

set(CMAKE_CXX_STANDARD 17)

add_subdirectory(${PICO_SDK_PATH}/src/common/boot_uf2 boot_uf2_headers)
add_subdirectory(${REPO_DIR}/lib/can_bus/canmore_protocol canmore_protocol)

add_executable(can_bl_tool
	main.cpp
	UF2.cpp
	FlashUI.cpp
	canmore_cpp/Discovery.cpp
	canmore_cpp/CANDiscovery.cpp
	canmore_cpp/EthernetDiscovery.cpp
	canmore_cpp/RegMappedClient.cpp
	canmore_cpp/RegMappedCANClient.cpp
	canmore_cpp/RegMappedEthernetClient.cpp
	canmore_cpp/BootloaderClient.cpp
)

# Copy in DeviceList
set(DEVICE_DEFINITIONS_FILE DeviceList.jsonc)
configure_file(${DEVICE_DEFINITIONS_FILE} ${DEVICE_DEFINITIONS_FILE} COPYONLY)
target_compile_definitions(can_bl_tool PUBLIC DEVICE_DEFINITIONS_FILE="${DEVICE_DEFINITIONS_FILE}")

target_compile_options(can_bl_tool PUBLIC -Wall -Wextra)

target_link_libraries(can_bl_tool
    canmore_protocol
	boot_uf2_headers
)
