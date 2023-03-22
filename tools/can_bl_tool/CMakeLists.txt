cmake_minimum_required(VERSION 3.12)

# Include local files
execute_process(COMMAND git rev-parse --show-cdup
	WORKING_DIRECTORY "${CMAKE_CURRENT_LIST_DIR}"
	OUTPUT_VARIABLE relative_path_to_repository_root OUTPUT_STRIP_TRAILING_WHITESPACE)
get_filename_component(REPO_DIR "${CMAKE_CURRENT_LIST_DIR}/${relative_path_to_repository_root}" ABSOLUTE)
set(PICO_SDK_PATH ${REPO_DIR}/lib/pico-sdk)

project(caN_bl_tool)

set(CMAKE_CXX_STANDARD 14)

add_subdirectory(${PICO_SDK_PATH}/src/common/boot_uf2 boot_uf2_headers)
add_subdirectory(${REPO_DIR}/lib/can_bus/canmore_protocol canmore_protocol)

add_executable(can_bl_tool main.c reg_mapped_client_linux.c)
target_link_libraries(can_bl_tool
    canmore_protocol
)