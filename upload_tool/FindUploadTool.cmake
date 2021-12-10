# Builds the UploadTool executable
#
# This will define the following variables
#
#    UPLOADTOOL_FOUND
#
# and the following imported targets
#
#     UPLOADTOOL_TARGET
#

if (NOT UPLOADTOOL_FOUND)
    # todo we would like to use pckgconfig to look for it first
    # see https://pabloariasal.github.io/2018/02/19/its-time-to-do-cmake-right/

    # Check if dependencies available, if not just don't build
    list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_LIST_DIR}/cmake)

    find_package(LIBUSB)
    find_package(udev)
    if (NOT LIBUSB_FOUND)
        message(FATAL_ERROR "upload-tool cannot be built because libUSB is not found")
    elseif (NOT UDEV_FOUND)
        message(FATAL_ERROR "upload-tool cannot be built because libudev is not found")
    else()
        include(ExternalProject)

        set(UPLOADTOOL_SOURCE_DIR ${REPO_DIR}/upload_tool)
        set(UPLOADTOOL_BINARY_DIR ${CMAKE_BINARY_DIR}/upload_tool)

        set(UPLOADTOOL_BUILD_TARGET UploadToolBuild)
        set(UPLOADTOOL_TARGET UploadTool)

        if (NOT TARGET ${UPLOADTOOL_BUILD_TARGET})
            ExternalProject_Add(${UPLOADTOOL_BUILD_TARGET}
                    PREFIX uploadtool SOURCE_DIR ${UPLOADTOOL_SOURCE_DIR}
                    BINARY_DIR ${UPLOADTOOL_BINARY_DIR}
                    BUILD_ALWAYS 1 # force dependency checking
                    INSTALL_COMMAND ""
                    )
        endif()

        set(UPLOADTOOL_EXECUTABLE ${UPLOADTOOL_BINARY_DIR}/upload-tool)
        if(NOT TARGET ${UPLOADTOOL_TARGET})
            add_executable(${UPLOADTOOL_TARGET} IMPORTED)
        endif()
        set_property(TARGET ${UPLOADTOOL_TARGET} PROPERTY IMPORTED_LOCATION
                ${UPLOADTOOL_EXECUTABLE})

        add_dependencies(${UPLOADTOOL_TARGET} ${UPLOADTOOL_BUILD_TARGET})
        set(UPLOADTOOL_FOUND 1)
    endif()
endif()
