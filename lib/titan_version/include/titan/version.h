#ifndef TITAN__VERSION_H_
#define TITAN__VERSION_H_

/**
 * @file titan/version.h
 *
 * @brief Versioning for titan firmware.
 *
 * Collects data from the build environemnt to determine version/releasey type version information.
 */

/**
 * @brief Enum of valid release types for versioning
 *
 * The reason behind these release types is to know what type of code is running
 * a given microcontroller.
 *
 * The version (should) appear during a boot, so in the event of crashes or bugs,
 * it should be clear what state the code is running on it at a glance
 * (especially when using safety for crash reporting).
 *
 * The second is when communicating with ROS. In the event prototype code was flashed
 * to a microcontroller and is working with ROS, diagnostics can report a version warning
 * since prototype code should never be running in an actual environment.
 */
enum version_release_type {
    /**
     * @brief Debug build: Built with extra debugging information.
     * This build may run slower or have timing issues due to the extra checks and overhead.
     * This type firmware should not run in production
     */
    DEBUG = 0,
    /**
     * @brief Development build.
     * This type is caused by the repository being dirty
     */
    DEV = 1,
    /**
     * @brief Clean build.
     * This type build has a clean working tree, allowing for easy rollback to this version
     */
    CLEAN = 2,
    /**
     * @brief Tagged build.
     * This type build has had the git commit tagged, meaning that it is (probably) a stable version
     */
    TAGGED = 3
};

/**
 * @brief The Major Version Number of the software
 * This should be incremented for any large or breaking changes to the program
 */
extern const int MAJOR_VERSION;
/**
 * @brief The Minor Version Number of the software
 * This should be incremented for any significant changes to the program which
 * differentiate it from the previous version
 */
extern const int MINOR_VERSION;

/**
 * @brief The Release Type of the software
 * See the `version_release_type` enum's docs for more info
 */
extern const enum version_release_type RELEASE_TYPE;

/**
 * @brief The full build tag of the software as a string
 * Should be able to uniquely identify builds of the program
 */
extern const char * const FULL_BUILD_TAG;

#endif