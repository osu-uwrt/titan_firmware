cmake_minimum_required(VERSION 3.12)

# Include local files
execute_process(COMMAND git rev-parse --show-cdup
	WORKING_DIRECTORY "${CMAKE_CURRENT_LIST_DIR}"
	OUTPUT_VARIABLE relative_path_to_repository_root OUTPUT_STRIP_TRAILING_WHITESPACE)
get_filename_component(REPO_DIR "${CMAKE_CURRENT_LIST_DIR}/${relative_path_to_repository_root}" ABSOLUTE)
set(PICO_SDK_PATH ${REPO_DIR}/lib/pico-sdk)

project(upload-tool)

set(CMAKE_CXX_STANDARD 14)

list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_LIST_DIR}/cmake)

find_package(LIBUSB)
find_package(udev)
if (NOT LIBUSB_FOUND)
    message(FATAL_ERROR "upload-tool cannot be built because libUSB is not found")
elseif (NOT UDEV_FOUND)
    message(FATAL_ERROR "upload-tool cannot be built because libudev is not found")
else()
    add_subdirectory(picoboot_connection)
    add_subdirectory(${PICO_SDK_PATH}/src/common/pico_binary_info pico_binary_info)
    add_subdirectory(${PICO_SDK_PATH}/src/common/boot_uf2 boot_uf2_headers)
    add_subdirectory(${PICO_SDK_PATH}/src/common/boot_picoboot boot_picoboot_headers)
    add_subdirectory(${PICO_SDK_PATH}/src/common/pico_usb_reset_interface pico_usb_reset_interface)
    add_subdirectory(${PICO_SDK_PATH}/src/host/pico_platform pico_platform)

    # add_executable(port-lookup port_lookup_tool.c discover.c board_definitions.c)

    # target_include_directories(port-lookup PRIVATE ${LIBUSB_INCLUDE_DIR} ${UDEV_INCLUDE_DIRS})
    # target_link_libraries(port-lookup 
    #     pico_binary_info
    #     boot_uf2_headers
    #     boot_picoboot_headers
    #     pico_platform_headers
    #     pico_usb_reset_interface_headers
    #     picoboot_connection
    #     picoboot_connection_cxx
    #     ${LIBUSB_LIBRARIES}
    #     ${UDEV_LIBRARIES}
    # )

    add_executable(upload-tool main.c discover.c reset.c serial.c board_definitions.c upload.cpp)
    target_include_directories(upload-tool PRIVATE ${LIBUSB_INCLUDE_DIR} ${UDEV_INCLUDE_DIRS})
    target_link_libraries(upload-tool
        pico_binary_info
        boot_uf2_headers
        boot_picoboot_headers
        pico_platform_headers
        pico_usb_reset_interface_headers
        picoboot_connection
        picoboot_connection_cxx
        ${LIBUSB_LIBRARIES}
        ${UDEV_LIBRARIES}
    )
endif()
