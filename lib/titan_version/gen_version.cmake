execute_process(COMMAND git log --pretty=format:'%h' -n 1
                OUTPUT_VARIABLE GIT_REV
                ERROR_QUIET)

# Check whether we got any revision (which isn't
# always the case, e.g. when someone downloaded a zip
# file from Github instead of a checkout)
if ("${GIT_REV}" STREQUAL "")
    set(GIT_REV "None")
    set(GIT_DIFF "")
    set(GIT_TAG "")
    set(GIT_BRANCH "None")
else()
    execute_process(
        COMMAND bash -c "(git diff --quiet --exit-code && test -z \"$(git ls-files $(git rev-parse --show-toplevel) --exclude-standard --others)\") || echo -dirty"
        OUTPUT_VARIABLE GIT_DIFF)
    execute_process(
        COMMAND git rev-parse --abbrev-ref HEAD
        OUTPUT_VARIABLE GIT_BRANCH)
    execute_process(
        COMMAND git describe --exact-match --tags HEAD
        RESULT_VARIABLE GIT_TAG_RESULT
        OUTPUT_VARIABLE GIT_TAG
        ERROR_QUIET
    )

    if ("${GIT_TAG_RESULT}" EQUAL 0)
        string(STRIP "${GIT_TAG}" GIT_TAG)
        set(GIT_TAG " (${GIT_TAG})")
    else()
        set(GIT_TAG "")
    endif()

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

string(TOLOWER "${CMAKE_BUILD_TYPE}" BUILD_TYPE_LOWER)

if ("${BUILD_TYPE_LOWER}" STREQUAL "debug")
    # Debug builds have special string appended at end of version string
    set(VER_RELEASE_TYPE "_debug")
    set(RELEASE_TYPE "DEBUG")
elseif (NOT "${GIT_DIFF}" STREQUAL "")
    # Dev builds are generated when the working tree is dirty
    set(VER_RELEASE_TYPE "")
    set(RELEASE_TYPE "DEV")
elseif ("${GIT_TAG}" STREQUAL "")
    # Clean builds are builds based on a clean repo without a git tag
    set(VER_RELEASE_TYPE "")
    set(RELEASE_TYPE "CLEAN")
else()
    # Tagged builds are based on a clean repo with a git tag
    set(VER_RELEASE_TYPE "")
    set(RELEASE_TYPE "TAGGED")
endif()

if ("${ROBOT}" STREQUAL "")
    set(TARGET_ENV "${BOARD}")
else()
    set(TARGET_ENV "${ROBOT}/${BOARD}")
endif()

set(VERSION "#if !defined(PICO_PROGRAM_NAME) && defined(PICO_TARGET_NAME)
#define PICO_PROGRAM_NAME PICO_TARGET_NAME
#endif
#include \"titan/version.h\"
#define VERSION_TAG \"${MAJOR_VERSION}.${MINOR_VERSION}${VER_RELEASE_TYPE}-${TARGET_ENV} ${GIT_BRANCH}/${GIT_REV}${GIT_DIFF}${GIT_TAG} ${BUILD_USER}@${BUILD_HOST} ${BUILD_TIMESTAMP}\"
const int MAJOR_VERSION = ${MAJOR_VERSION};
const int MINOR_VERSION = ${MINOR_VERSION};
const enum version_release_type RELEASE_TYPE = ${RELEASE_TYPE};
const char * const FULL_BUILD_TAG=PICO_PROGRAM_NAME \" \" VERSION_TAG;

#if !PICO_NO_BINARY_INFO && !PICO_NO_PROGRAM_INFO
#include \"pico/binary_info.h\"
#include \"titan/binary_info.h\"
bi_decl(bi_program_version_string(VERSION_TAG));
bi_decl(bi_titan_version(MAJOR_VERSION, MINOR_VERSION, RELEASE_TYPE));
#endif")

if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/build_version.c)
    file(READ ${CMAKE_CURRENT_SOURCE_DIR}/build_version.c VERSION_)
else()
    set(VERSION_ "")
endif()

if (NOT "${VERSION}" STREQUAL "${VERSION_}")
    file(WRITE ${CMAKE_CURRENT_SOURCE_DIR}/build_version.c "${VERSION}")
endif()
