cmake_minimum_required(VERSION 3.12)

# Include local files
execute_process(COMMAND git rev-parse --show-cdup
	WORKING_DIRECTORY "${CMAKE_CURRENT_LIST_DIR}"
	OUTPUT_VARIABLE relative_path_to_repository_root OUTPUT_STRIP_TRAILING_WHITESPACE)
get_filename_component(REPO_DIR "${CMAKE_CURRENT_LIST_DIR}/${relative_path_to_repository_root}" ABSOLUTE)
set(PICO_SDK_PATH ${REPO_DIR}/lib/pico-sdk)

project(upload_tool)

set(CMAKE_CXX_STANDARD 17)

# Add common titan libraries
add_subdirectory(../lib titan_tools_lib)

# Create upload tool
add_executable(upload_tool
	main.cpp
	UF2.cpp
	FlashUI.cpp
)

target_compile_options(upload_tool PUBLIC -Wno-psabi -Wall -Wextra)

target_link_libraries(upload_tool
        binary_info_cpp
        canmore_cpp
        pico_usb
        rp2040_flash_itf
)

install(TARGETS upload_tool RUNTIME DESTINATION bin)
