cmake_minimum_required(VERSION 3.12)

# Required libraries for BinaryInfo
add_subdirectory(${PICO_SDK_PATH}/src/common/pico_binary_info pico_binary_info)
add_subdirectory(${PICO_SDK_PATH}/src/common/boot_uf2 boot_uf2_headers)
add_subdirectory(${REPO_DIR}/lib/titan_binary_info titan_binary_info)

# Define cmake library
add_library(binary_info_cpp INTERFACE)

target_sources(binary_info_cpp INTERFACE
	${CMAKE_CURRENT_LIST_DIR}/BinaryInfo.cpp
	${CMAKE_CURRENT_LIST_DIR}/BinaryInfoDisplay.cpp
)

target_include_directories(binary_info_cpp INTERFACE ${CMAKE_CURRENT_LIST_DIR}/include/)

target_link_libraries(binary_info_cpp INTERFACE
	boot_uf2_headers
	pico_binary_info
	titan_binary_info
)
