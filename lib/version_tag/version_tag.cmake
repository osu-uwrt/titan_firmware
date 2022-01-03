cmake_minimum_required(VERSION 3.13)

set(_script_dir ${CMAKE_CURRENT_LIST_DIR} CACHE INTERNAL "")

function(generate_version_tag target major_version minor_version release_type)
    # Valid Release Types are the enums in build_version.h

    # Add a custom command that produces build_version.cpp, plus
    # a dummy output that's not actually produced, in order
    # to force gen_version.cmake to always be re-run before the build
    ADD_CUSTOM_COMMAND(
        OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/build_version.c
            ${CMAKE_CURRENT_BINARY_DIR}/_build_version.c
        COMMAND ${CMAKE_COMMAND} -DMAJOR_VERSION=${major_version} -DMINOR_VERSION=${minor_version} -DRELEASE_TYPE=${release_type} -DBOARD=${PICO_BOARD} 
                -P ${_script_dir}/gen_version.cmake)

    target_include_directories(${target} PRIVATE ${_script_dir}/include)
    target_sources(${target} PRIVATE ${CMAKE_CURRENT_BINARY_DIR}/build_version.c)
endfunction()