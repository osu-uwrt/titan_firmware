cmake_minimum_required(VERSION 3.12)

# Include local files
execute_process(COMMAND git rev-parse --show-cdup
    WORKING_DIRECTORY "${CMAKE_CURRENT_LIST_DIR}"
    OUTPUT_VARIABLE relative_path_to_repository_root OUTPUT_STRIP_TRAILING_WHITESPACE)
get_filename_component(REPO_DIR "${CMAKE_CURRENT_LIST_DIR}/${relative_path_to_repository_root}" ABSOLUTE)

project(canmore_can)

set(CMAKE_CXX_STANDARD 17)

# Only needs canmore_cpp
set(NO_AMENT 1)
add_subdirectory(${REPO_DIR}/lib/titan_canmore titan_canmore)

# libusb to list adapters
find_package(PkgConfig REQUIRED)
pkg_check_modules(LIBUSB REQUIRED libusb-1.0)

add_executable(canmore_can
    main.cpp
)

target_compile_options(canmore_can PUBLIC -Wno-psabi -Wall -Wextra)
target_include_directories(canmore_can PRIVATE ${LIBUSB_INCLUDE_DIRS})

target_link_libraries(canmore_can
    canmore_cpp
    ${LIBUSB_LINK_LIBRARIES}
)

install(TARGETS canmore_can RUNTIME DESTINATION bin)
