execute_process(COMMAND git log --pretty=format:'%h' -n 1
                OUTPUT_VARIABLE GIT_REV
                ERROR_QUIET)

# Check whether we got any revision (which isn't
# always the case, e.g. when someone downloaded a zip
# file from Github instead of a checkout)
if ("${GIT_REV}" STREQUAL "")
    set(GIT_REV "None")
    set(GIT_DIFF "")
    set(GIT_BRANCH "None")
else()
    execute_process(
        COMMAND bash -c "(git diff --quiet --exit-code && test -z \"$(git ls-files --others)\") || echo -dirty"
        OUTPUT_VARIABLE GIT_DIFF)
    execute_process(
        COMMAND git rev-parse --abbrev-ref HEAD
        OUTPUT_VARIABLE GIT_BRANCH)

    string(STRIP "${GIT_REV}" GIT_REV)
    string(SUBSTRING "${GIT_REV}" 1 7 GIT_REV)
    string(STRIP "${GIT_DIFF}" GIT_DIFF)
    string(STRIP "${GIT_BRANCH}" GIT_BRANCH)
endif()

execute_process(
    COMMAND whoami
    OUTPUT_VARIABLE BUILD_USER)
execute_process(
    COMMAND hostname
    OUTPUT_VARIABLE BUILD_HOST)
execute_process(
    COMMAND date -Iseconds
    OUTPUT_VARIABLE BUILD_TIMESTAMP
)
string(STRIP "${BUILD_USER}" BUILD_USER)
string(STRIP "${BUILD_HOST}" BUILD_HOST)
string(STRIP "${BUILD_TIMESTAMP}" BUILD_TIMESTAMP)

if (("${RELEASE_TYPE}" STREQUAL "STABLE") AND (NOT "${GIT_DIFF}" STREQUAL ""))
    message(WARNING "\nRelease type set to STABLE but repository is dirty\nSetting release type to DEV")
    set(RELEASE_TYPE "DEV")
endif()

if ("${RELEASE_TYPE}" STREQUAL "STABLE")
    # STABLE doesn't have anything appended to the end of the string
    set(VER_RELEASE_TYPE "")
else()
    string(TOLOWER "_${RELEASE_TYPE}" VER_RELEASE_TYPE)
endif()

set(VERSION "#if !defined(PICO_PROGRAM_NAME) && defined(PICO_TARGET_NAME)
#define PICO_PROGRAM_NAME PICO_TARGET_NAME
#endif
#include \"build_version.h\"
#define VERSION_TAG \"${MAJOR_VERSION}.${MINOR_VERSION}${VER_RELEASE_TYPE}-${ROBOT}/${BOARD} ${GIT_BRANCH}/${GIT_REV}${GIT_DIFF} ${BUILD_USER}@${BUILD_HOST} ${BUILD_TIMESTAMP}\"
const int MAJOR_VERSION = ${MAJOR_VERSION};
const int MINOR_VERSION = ${MINOR_VERSION};
const enum version_release_type RELEASE_TYPE = ${RELEASE_TYPE};
const char * const FULL_BUILD_TAG=PICO_PROGRAM_NAME \" \" VERSION_TAG;

#if !PICO_NO_BINARY_INFO && !PICO_NO_PROGRAM_INFO
#include \"pico/binary_info.h\"
bi_decl(bi_program_version_string(VERSION_TAG))
#endif")

if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/build_version.c)
    file(READ ${CMAKE_CURRENT_SOURCE_DIR}/build_version.c VERSION_)
else()
    set(VERSION_ "")
endif()

if (NOT "${VERSION}" STREQUAL "${VERSION_}")
    file(WRITE ${CMAKE_CURRENT_SOURCE_DIR}/build_version.c "${VERSION}")
endif()