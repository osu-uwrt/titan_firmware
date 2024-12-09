# Finds (or builds) the BOOTUF2CAT executable
#
# This will define the following variables
#
#    BOOTUF2CAT_FOUND
#
# and the following imported targets
#
#     BOOTUF2CAT
#

if (NOT BOOTUF2CAT_FOUND)
    # todo we would like to use pckgconfig to look for it first
    # see https://pabloariasal.github.io/2018/02/19/its-time-to-do-cmake-right/

    include(ExternalProject)

    set(BOOTUF2CAT_SOURCE_DIR ${CMAKE_CURRENT_LIST_DIR}/boot_uf2_cat)
    set(BOOTUF2CAT_BINARY_DIR ${CMAKE_BINARY_DIR}/boot_uf2_cat)

    set(BOOTUF2CAT_BUILD_TARGET BOOTUF2CATBuild)
    set(BOOTUF2CAT_TARGET BOOTUF2CAT)

    if (NOT TARGET ${BOOTUF2CAT_BUILD_TARGET})
        pico_message_debug("BOOTUF2CAT will need to be built")
        ExternalProject_Add(${BOOTUF2CAT_BUILD_TARGET}
                PREFIX boot_uf2_cat SOURCE_DIR ${BOOTUF2CAT_SOURCE_DIR}
                BINARY_DIR ${BOOTUF2CAT_BINARY_DIR}
                BUILD_ALWAYS 1 # force dependency checking
                INSTALL_COMMAND ""
                )
    endif()

    set(BOOTUF2CAT_EXECUTABLE ${BOOTUF2CAT_BINARY_DIR}/boot_uf2_cat)
    if(NOT TARGET ${BOOTUF2CAT_TARGET})
        add_executable(${BOOTUF2CAT_TARGET} IMPORTED)
    endif()
    set_property(TARGET ${BOOTUF2CAT_TARGET} PROPERTY IMPORTED_LOCATION
            ${BOOTUF2CAT_EXECUTABLE})

    add_dependencies(${BOOTUF2CAT_TARGET} ${BOOTUF2CAT_BUILD_TARGET})
    set(BOOTUF2CAT_FOUND 1)
endif()
