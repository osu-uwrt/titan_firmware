set(TITAN_BOOTLOADER_SCRIPT_DIR ${CMAKE_CURRENT_LIST_DIR} CACHE INTERNAL "")
set(TITAN_BOOTLOADER_BINARY_DIR ${CMAKE_CURRENT_BINARY_DIR} CACHE INTERNAL "")

function(titan_enable_bootloader TARGET TYPE)
    ExternalProject_Add (
        titan_bootloader
        SOURCE_DIR ${TITAN_BOOTLOADER_SCRIPT_DIR}
        PREFIX ${TITAN_BOOTLOADER_BINARY_DIR}/titan_bootloader
        CMAKE_ARGS -DCMAKE_BUILD_TYPE=Release -DBOOTLOADER_INTERFACE=${TYPE} -DPICO_BOARD=${PICO_BOARD} -DUWRT_ROBOT=${UWRT_ROBOT}
        INSTALL_COMMAND ""
    )

    ExternalProject_Get_Property(titan_bootloader BINARY_DIR)

    set(UF2_COMMON_NAME "$<IF:$<BOOL:$<TARGET_PROPERTY:${TARGET},OUTPUT_NAME>>,$<TARGET_PROPERTY:${TARGET},OUTPUT_NAME>,$<TARGET_PROPERTY:${TARGET},NAME>>")
    set(UF2_OTA_FILE "${UF2_COMMON_NAME}_ota.uf2")
    set(UF2_COMBINED_FILE "${UF2_COMMON_NAME}_with_bl.uf2")

    set_target_properties(${TARGET} PROPERTIES
        BOOTLOADER_ENABLED 1
        BOOTLOADER_BL_UF2_FILE "${BINARY_DIR}/titan_bootloader.uf2")

    # Make sure pico doesn't spit out a UF2
    # This is very cursed, where the very order of the set statement will determine whether UF2s are made
    # Since we're making our own UF2, we need to tell the pico sdk not to make one
    # However if pico_add_extra_outputs was already called, this set has no effect, so we need to first check if
    # pico_add_extra_outputs has been called. There isn't really a good way to do this, but the best appears to be by checking compile definitions
    # Note if PICO_NO_TARGET_NAME is set, this check won't work, but there isn't really a better way to catch the extra outputs call
    get_target_property(BOOTLOADER_TARGET_DEFS ${TARGET} COMPILE_DEFINITIONS)
    if(BOOTLOADER_TARGET_DEFS MATCHES "^PICO_TARGET_NAME=" OR BOOTLOADER_TARGET_DEFS MATCHES ";PICO_TARGET_NAME=")
        MESSAGE(FATAL_ERROR "pico_add_extra_outputs must be called after titan_enable_bootloader" )
    endif()

    # Now that we've checked that the target doesn't have extra outputs generated yet, and we've generated the bootloader
    # (so it gets a uf2 file) disable uf2 file generation for the sdk
    set(PICO_NO_UF2 1 PARENT_SCOPE)

    # Declare bootloader as dependency
    add_dependencies(${TARGET} titan_bootloader)

    # Configure the project to use the bootloader app linker script (so it has space for the bootloader)
	pico_set_linker_script(${TARGET} ${TITAN_BOOTLOADER_SCRIPT_DIR}/bootloader_runtime/memmap_app.ld)

    # Prep for uf2 generation
    if (NOT BOOTUF2CAT_FOUND)
        set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${TITAN_BOOTLOADER_SCRIPT_DIR})
        find_package(BOOTUF2CAT)
    endif()
    if (NOT ELF2UF2_FOUND)
        set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${PICO_SDK_PATH}/tools)
        find_package(ELF2UF2)
    endif()

    # Build the uf2 file for the project (needs to be different to suffix it with _ota)
    add_custom_command(TARGET ${TARGET} POST_BUILD
            COMMAND ELF2UF2 "$<TARGET_FILE:${TARGET}>" "${UF2_OTA_FILE}")

    # Generate the combined uf2 file for the project (to allow usb flashing)
    add_custom_command(TARGET ${TARGET} POST_BUILD
        COMMAND BOOTUF2CAT
            "$<TARGET_PROPERTY:${TARGET},BOOTLOADER_BL_UF2_FILE>"
            "${UF2_OTA_FILE}"
            "${UF2_COMBINED_FILE}")
endfunction()