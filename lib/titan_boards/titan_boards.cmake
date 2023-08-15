# Add boards folder to PICO_BOARD_HEADER_DIRS
list(APPEND PICO_BOARD_HEADER_DIRS ${CMAKE_CURRENT_LIST_DIR}/include/boards)
list(APPEND PICO_INCLUDE_DIRS ${CMAKE_CURRENT_LIST_DIR}/include) # so boards/foo.h can be explicitly included

if (DEFINED UWRT_ROBOT)
    set(UWRT_ROBOT_HEADER_FILE ${CMAKE_CURRENT_LIST_DIR}/include/robots/${UWRT_ROBOT}.h)

    if (EXISTS ${UWRT_ROBOT_HEADER_FILE})
        message("Using robot configuration from ${UWRT_ROBOT_HEADER_FILE}")
        list(APPEND PICO_CONFIG_HEADER_FILES ${UWRT_ROBOT_HEADER_FILE})
    else()
        message(FATAL_ERROR "Unable to find definition of robot '${UWRT_ROBOT}' (specified by UWRT_ROBOT) at '${UWRT_ROBOT_HEADER_FILE}'\n")
    endif()
endif()
