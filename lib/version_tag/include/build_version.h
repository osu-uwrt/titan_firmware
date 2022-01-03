#ifndef _BUILD_VERSION_H
#define _BUILD_VERSION_H

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
     * @brief Prototype build, under heavy development
     * Critical features could be missing and no stable operation is gaurenteed
     */
    PROTO = 0,
    /**
     * @brief Development build, under development
     * Most features should be present and working, but there could still be bugs
     */
    DEV = 1,
    /**
     * @brief Stable build, development finished
     * All features should be working, code should be tested to run as intended
     */
    STABLE = 2,
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