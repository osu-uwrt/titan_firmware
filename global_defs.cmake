cmake_minimum_required(VERSION 3.13)

# Include sdk
set(PICO_SDK_PATH ${REPO_DIR}/lib/pico-sdk)
include(${PICO_SDK_PATH}/external/pico_sdk_import.cmake)

# Add uwrt custom boards
include(${REPO_DIR}/lib/uwrt_boards/uwrt_boards.cmake)

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

function(titan_firmware_init)
    # Make relwithdebuginfo actually like Release
    set(CMAKE_${LANG}_FLAGS_RELWITHDEBINFO "-O3 -DNDEBUG -g")

    # Setup sdk
    set(CMAKE_C_STANDARD 11)
    set(CMAKE_CXX_STANDARD 17)
    pico_sdk_init()

    # Define all custom libraries
    include(${REPO_DIR}/bootloader/enable_bootloader.cmake)
    add_subdirectory(${REPO_DIR}/lib/ titan_lib)
endfunction()
