# Add boards folder to PICO_BOARD_HEADER_DIRS
list(APPEND PICO_BOARD_HEADER_DIRS ${CMAKE_CURRENT_LIST_DIR}/include/boards)
list(APPEND PICO_INCLUDE_DIRS ${CMAKE_CURRENT_LIST_DIR}/include) # so boards/foo.h can be explicitly included

# Include robot definition if requested
if (DEFINED UWRT_ROBOT)
    set(UWRT_ROBOT_HEADER_FILE ${CMAKE_CURRENT_LIST_DIR}/include/robots/${UWRT_ROBOT}.h)

    if (EXISTS ${UWRT_ROBOT_HEADER_FILE})
        message("Using robot configuration from ${UWRT_ROBOT_HEADER_FILE}")
        list(APPEND PICO_CONFIG_HEADER_FILES ${UWRT_ROBOT_HEADER_FILE})
    else()
        message(FATAL_ERROR "Unable to find definition of robot '${UWRT_ROBOT}' (specified by UWRT_ROBOT) at '${UWRT_ROBOT_HEADER_FILE}'\n")
    endif()
endif()

# Include canmore client ids to all files so each board definition can directly refer to where it belongs to
set(CANMORE_CLIENT_ID_HEADER "${REPO_DIR}/lib/titan_canmore/canmore/include/canmore/client_ids.h")
if (NOT EXISTS "${CANMORE_CLIENT_ID_HEADER}")
    message(FATAL_ERROR "Unable to locate client id definition header: ${CANMORE_CLIENT_ID_HEADER}")
endif()
list(APPEND PICO_CONFIG_HEADER_FILES ${CANMORE_CLIENT_ID_HEADER})
