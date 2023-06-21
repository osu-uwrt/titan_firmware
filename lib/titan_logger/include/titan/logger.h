#ifndef TITAN__LOGGER_H_
#define TITAN__LOGGER_H_

#include <assert.h>
#include <stdarg.h>
#include <stdio.h>

/**
 * @file titan/logger.h
 *
 * @brief A basic logging library to allow for more informative logs while debugging, and limiting of debug information
 * to allow for faster execution in non-debug builds.
 *
 */

// ========================================
// Exported Functions
// ========================================

typedef void (*custom_logger_cb_t)(void* args, const int log_level, const char * unit, const char * filename, const int line, const char * const function, const char * const fmt, va_list argp);
extern void titan_logger_set_global_log_level(int log_level);
extern void titan_logger_set_custom_logger_callback(custom_logger_cb_t callback, void* args);

// Aditional Functions:
// LOG_DEBUG(...)
// LOG_INFO(...)
// LOG_WARN(...)
// LOG_ERROR(...)
// LOG_FATAL(...)

// ========================================
// Local Unit Configuration
// ========================================
/* These can be undefined and redefined to change local log settings
 * LOGGING_UNIT_NAME makes it easier to discern which code unit created the debug message
 * LOGGING_UNIT_LOCAL_LEVEL is useful when combined with other preprocessor values to hide extra debug information
 * unless required.
 *
 * -----For example-----
 * #include "titan/logger.h"
 *
 * #undef LOGGING_UNIT_NAME
 * #define LOGGING_UNIT_NAME "My Unit Name"
 *
 * #undef LOGGING_UNIT_LOCAL_LEVEL
 * #define LEVEL_WARN
 *
 * void my_function(void) {
 *     LOG_INFO("This will be hidden");
 *     LOG_WARN("This will not");
 * }
 */

#define LOGGING_UNIT_NAME "No Unit"
#define LOGGING_UNIT_LOCAL_LEVEL TITAN_LOGGER_DEFAULT_LEVEL

// ========================================
// Global Configuration Options (set in CMakeLists)
// ========================================

#ifndef NDEBUG
// Include deubg information by default if assertions are enabled (meaning debug)

// PICO_CONFIG: TITAN_LOGGER_MIN_SEVERITY, Specifies the minimum logger severity (after which the log statements will be optimized out), type=string, group=titan_logger
#ifndef TITAN_LOGGER_MIN_SEVERITY
#define TITAN_LOGGER_MIN_SEVERITY LEVEL_DEBUG
#endif

// PICO_CONFIG: TITAN_LOGGER_DEFAULT_LEVEL, Specifies the default logger severity, which can be lowered to TITAN_LOGGER_MIN_SEVERITY using titan_logger_set_global_log_level, type=bool, group=titan_logger
#ifndef TITAN_LOGGER_DEFAULT_LEVEL
#define TITAN_LOGGER_DEFAULT_LEVEL TITAN_LOGGER_MIN_SEVERITY
#endif

// PICO_CONFIG: TITAN_LOGGER_USE_COLOR_CODES, Enables color in logger output, type=bool, group=titan_logger
#ifndef TITAN_LOGGER_USE_COLOR_CODES
#define TITAN_LOGGER_USE_COLOR_CODES 1
#endif

// PICO_CONFIG: TITAN_LOGGER_PRINT_SOURCE_LOCATION, Prints the file/line of the log statement in logger output, type=bool, group=titan_logger
#ifndef TITAN_LOGGER_PRINT_SOURCE_LOCATION
#define TITAN_LOGGER_PRINT_SOURCE_LOCATION 1
#endif

#else
// Disable debug information if assertions are disabled (meaning release)
#ifndef TITAN_LOGGER_MIN_SEVERITY
#define TITAN_LOGGER_MIN_SEVERITY LEVEL_INFO
#endif

#ifndef TITAN_LOGGER_DEFAULT_LEVEL
#define TITAN_LOGGER_DEFAULT_LEVEL TITAN_LOGGER_MIN_SEVERITY
#endif

#ifndef TITAN_LOGGER_USE_COLOR_CODES
#define TITAN_LOGGER_USE_COLOR_CODES 0
#endif

#ifndef TITAN_LOGGER_PRINT_SOURCE_LOCATION
#define TITAN_LOGGER_PRINT_SOURCE_LOCATION 0
#endif
#endif

// ========================================
// Internal Definitions
// ========================================

#define LEVEL_DEBUG 1
#define LEVEL_INFO  2
#define LEVEL_WARN  3
#define LEVEL_ERROR 4
#define LEVEL_FAULT 5   // NOTE: Special level for safety fault messages
#define LEVEL_FATAL 6
#define LEVEL_NO_LOGGING 7

#if TITAN_LOGGER_DEFAULT_LEVEL < TITAN_LOGGER_MIN_SEVERITY
#error Default logging level is below minimum severity
#endif

extern void titan_logger_log_common(const int log_level, const int local_log_level, const char * unit,
                                    const char * filename, const int line, const char * const function,
                                    const char * const fmt, ...)
    _ATTRIBUTE ((__format__ (__printf__, 7, 8)));

// Logging function definitions
#if TITAN_LOGGER_MIN_SEVERITY > LEVEL_DEBUG
#define LOG_DEBUG(...) do {} while(0)
#else
#if TITAN_LOGGER_PRINT_SOURCE_LOCATION
#define LOG_DEBUG(...) do {titan_logger_log_common(LEVEL_DEBUG, LOGGING_UNIT_LOCAL_LEVEL, LOGGING_UNIT_NAME, __FILE__, __LINE__, __func__, __VA_ARGS__);} while(0)
#else
#define LOG_DEBUG(...) do {titan_logger_log_common(LEVEL_DEBUG, LOGGING_UNIT_LOCAL_LEVEL, LOGGING_UNIT_NAME, "", 0, "", __VA_ARGS__);} while(0)
#endif
#endif

#if TITAN_LOGGER_MIN_SEVERITY > LEVEL_INFO
#define LOG_INFO(...) do {} while(0)
#else
#if TITAN_LOGGER_PRINT_SOURCE_LOCATION
#define LOG_INFO(...) do {titan_logger_log_common(LEVEL_INFO, LOGGING_UNIT_LOCAL_LEVEL, LOGGING_UNIT_NAME, __FILE__, __LINE__, __func__, __VA_ARGS__);}  while(0)
#else
#define LOG_INFO(...) do {titan_logger_log_common(LEVEL_INFO, LOGGING_UNIT_LOCAL_LEVEL, LOGGING_UNIT_NAME, "", 0, "", __VA_ARGS__);}  while(0)
#endif
#endif

#if TITAN_LOGGER_MIN_SEVERITY > LEVEL_WARN
#define LOG_WARN(...) do {}  while(0)
#else
#if TITAN_LOGGER_PRINT_SOURCE_LOCATION
#define LOG_WARN(...) do {titan_logger_log_common(LEVEL_WARN, LOGGING_UNIT_LOCAL_LEVEL, LOGGING_UNIT_NAME, __FILE__, __LINE__, __func__, __VA_ARGS__);}  while(0)
#else
#define LOG_WARN(...) do {titan_logger_log_common(LEVEL_WARN, LOGGING_UNIT_LOCAL_LEVEL, LOGGING_UNIT_NAME, "", 0, "", __VA_ARGS__);}  while(0)
#endif
#endif

#if TITAN_LOGGER_MIN_SEVERITY > LEVEL_ERROR
#define LOG_ERROR(...) do {}  while(0)
#else
#if TITAN_LOGGER_PRINT_SOURCE_LOCATION
#define LOG_ERROR(...) do {titan_logger_log_common(LEVEL_ERROR, LOGGING_UNIT_LOCAL_LEVEL, LOGGING_UNIT_NAME, __FILE__, __LINE__, __func__, __VA_ARGS__);}  while(0)
#else
#define LOG_ERROR(...) do {titan_logger_log_common(LEVEL_ERROR, LOGGING_UNIT_LOCAL_LEVEL, LOGGING_UNIT_NAME, "", 0, "", __VA_ARGS__);}  while(0)
#endif
#endif

#if TITAN_LOGGER_MIN_SEVERITY > LEVEL_FAULT
#define LOG_FAULT(...) do {}  while(0)
#else
#if TITAN_LOGGER_PRINT_SOURCE_LOCATION
#define LOG_FAULT(...) do {titan_logger_log_common(LEVEL_FAULT, LOGGING_UNIT_LOCAL_LEVEL, LOGGING_UNIT_NAME, __FILE__, __LINE__, __func__, __VA_ARGS__);}  while(0)
#else
#define LOG_FAULT(...) do {titan_logger_log_common(LEVEL_FAULT, LOGGING_UNIT_LOCAL_LEVEL, LOGGING_UNIT_NAME, "", 0, "", __VA_ARGS__);}  while(0)
#endif
#endif

#if TITAN_LOGGER_MIN_SEVERITY > LEVEL_FATAL
#define LOG_FATAL(...) do {}  while(0)
#else
#if TITAN_LOGGER_PRINT_SOURCE_LOCATION
#define LOG_FATAL(...) do {titan_logger_log_common(LEVEL_FATAL, LOGGING_UNIT_LOCAL_LEVEL, LOGGING_UNIT_NAME, __FILE__, __LINE__, __func__, __VA_ARGS__);}  while(0)
#else
#define LOG_FATAL(...) do {titan_logger_log_common(LEVEL_FATAL, LOGGING_UNIT_LOCAL_LEVEL, LOGGING_UNIT_NAME, "", 0, "", __VA_ARGS__);}  while(0)
#endif
#endif

#endif
