cmake_minimum_required(VERSION 3.13)

# Include sdk
set(PICO_SDK_PATH ${REPO_DIR}/lib/pico-sdk)
include(${PICO_SDK_PATH}/external/pico_sdk_import.cmake)

# Add uwrt custom boards
include(${REPO_DIR}/lib/uwrt_boards/uwrt_boards.cmake)

# Add versioning commands
include(${REPO_DIR}/lib/version_tag/version_tag.cmake)

# Define custom functions for assorted features
function(uwrt_use_upload_tool target)
    set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${REPO_DIR}/upload_tool)
    find_package(UploadTool)
    if (UPLOADTOOL_FOUND)
        add_custom_target(upload
                    COMMAND ${UPLOADTOOL_EXECUTABLE} -u -f $<IF:$<BOOL:$<TARGET_PROPERTY:${target},OUTPUT_NAME>>,$<TARGET_PROPERTY:${target},OUTPUT_NAME>,$<TARGET_PROPERTY:${target},NAME>>.uf2
                    COMMAND sleep 0.5
                    DEPENDS ${UPLOADTOOL_TARGET}
                    DEPENDS ${target})

        add_custom_target(serial
                    # By passing filename in it will auto-select the board for that given firmware
                    COMMAND ${UPLOADTOOL_EXECUTABLE} -m -w 5000 -f $<IF:$<BOOL:$<TARGET_PROPERTY:${target},OUTPUT_NAME>>,$<TARGET_PROPERTY:${target},OUTPUT_NAME>,$<TARGET_PROPERTY:${target},NAME>>.uf2
                    DEPENDS ${UPLOADTOOL_TARGET})

        add_custom_target(swd_flash
                    # By passing filename in it will auto-select the board for that given firmware
                    COMMAND ${REPO_DIR}/tools/swd_flash.sh $<IF:$<BOOL:$<TARGET_PROPERTY:${target},OUTPUT_NAME>>,$<TARGET_PROPERTY:${target},OUTPUT_NAME>,$<TARGET_PROPERTY:${target},NAME>>.elf
                    DEPENDS ${target})
    else()
        message( WARNING "Could not find upload tool. Custom make targets will not be available" )
    endif()
endfunction()

# Define all custom libraries
add_subdirectory(${REPO_DIR}/lib/async_i2c/ async_i2c)
add_subdirectory(${REPO_DIR}/lib/basic_logger/ basic_logger)
add_subdirectory(${REPO_DIR}/lib/basic_queue/ basic_queue)
add_subdirectory(${REPO_DIR}/lib/can_bus/ can_bus)
add_subdirectory(${REPO_DIR}/lib/dual_serial_stdio_usb/ dual_serial_stdio_usb_build)
add_subdirectory(${REPO_DIR}/lib/micro_ros_pico/ micro_ros_pico)
add_subdirectory(${REPO_DIR}/lib/safety/ safety)
add_subdirectory(${REPO_DIR}/lib/wiznet_ethernet/ wiznet_ethernet)
