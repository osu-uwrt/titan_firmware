cmake_minimum_required(VERSION 3.17)    # Required for CMAKE_CURRENT_FUNCTION_LIST_DIR

function(titan_use_upload_tool target)
    set(UPLOAD_TOOL_SRC_DIR "${CMAKE_CURRENT_FUNCTION_LIST_DIR}")
    set(UPLOAD_TOOL_BIN_DIR "${UPLOAD_TOOL_SRC_DIR}/build")
    set(UPLOAD_TOOL_BIN_NAME "upload_tool")
    set(UPLOAD_TOOL_PATH "${UPLOAD_TOOL_BIN_DIR}/${UPLOAD_TOOL_BIN_NAME}")

    set(UF2_BASENAME "$<IF:$<BOOL:$<TARGET_PROPERTY:${target},OUTPUT_NAME>>,$<TARGET_PROPERTY:${target},OUTPUT_NAME>,$<TARGET_PROPERTY:${target},NAME>>")
    set(UF2_NORMAL_FILE "${UF2_BASENAME}.uf2")      # Pico sdk normal uf2 files
    set(UF2_OTA_FILE "${UF2_BASENAME}_ota.uf2")
    set(UF2_COMBINED_FILE "${UF2_BASENAME}_with_bl.uf2")    # Full uf2 files with bootloader enabled

    # A scuffed way to make an external project only compile when the custom commands are used
    add_custom_target(upload_tool
                COMMAND cmake -S "${UPLOAD_TOOL_SRC_DIR}" -B "${UPLOAD_TOOL_BIN_DIR}"
                COMMAND cmake --build "${UPLOAD_TOOL_BIN_DIR}" --parallel)

    # Determines default uf2 filename whether bootloader is enabled or not
    set(UF2_DEFAULT_NAME "$<IF:$<BOOL:$<TARGET_PROPERTY:${target},BOOTLOADER_ENABLED>>,${UF2_COMBINED_FILE},${UF2_NORMAL_FILE}>")

    add_custom_target(upload
                COMMAND echo
                COMMAND "${UPLOAD_TOOL_PATH}" -f "${UF2_DEFAULT_NAME}"
                COMMAND echo
                DEPENDS upload_tool
                DEPENDS ${target})

    add_custom_target(info
                COMMAND echo
                COMMAND "${UPLOAD_TOOL_PATH}" -i "${UF2_DEFAULT_NAME}"
                COMMAND echo
                DEPENDS upload_tool
                DEPENDS ${target})

    add_custom_target(upload_ota
                COMMAND echo
                COMMAND "${UPLOAD_TOOL_PATH}" "${UF2_OTA_FILE}"
                COMMAND echo
                DEPENDS upload_tool
                DEPENDS ${target})

    add_custom_target(swd_flash
                COMMAND echo
                COMMAND "${UPLOAD_TOOL_PATH}" -f -o "${UF2_DEFAULT_NAME}"
                COMMAND echo
                DEPENDS upload_tool
                DEPENDS ${target})

endfunction()
