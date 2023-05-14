set(TITAN_VERSION_SCRIPT_DIR ${CMAKE_CURRENT_LIST_DIR} CACHE INTERNAL "")
set(TITAN_VERSION_BINARY_DIR ${CMAKE_CURRENT_BINARY_DIR} CACHE INTERNAL "")

function(titan_generate_version_tag target major_version minor_version)
    # Valid Release Types are the enums in titan/version.h

    # Add a custom command that produces build_version.cpp, plus
    # a dummy output that's not actually produced, in order
    # to force gen_version.cmake to always be re-run before the build
    ADD_CUSTOM_COMMAND(
        OUTPUT ${TITAN_VERSION_BINARY_DIR}/build_version.c
            ${TITAN_VERSION_BINARY_DIR}/_build_version.c
        WORKING_DIRECTORY ${TITAN_VERSION_BINARY_DIR}
        COMMAND ${CMAKE_COMMAND} -DMAJOR_VERSION=${major_version} -DMINOR_VERSION=${minor_version} -DBOARD=${PICO_BOARD} -DROBOT=${UWRT_ROBOT} -DCMAKE_BUILD_TYPE=${CMAKE_BUILD_TYPE}
                -P ${TITAN_VERSION_SCRIPT_DIR}/gen_version.cmake)

    target_include_directories(${target} PRIVATE ${TITAN_VERSION_SCRIPT_DIR}/include)
    target_sources(${target} PRIVATE ${TITAN_VERSION_BINARY_DIR}/build_version.c)
    target_link_libraries(${target} PUBLIC titan_binary_info)
endfunction()