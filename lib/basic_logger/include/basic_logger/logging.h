#ifndef _LOGGING__LOGGING_H
#define _LOGGING__LOGGING_H

#include <assert.h>
#include <stdarg.h>
#include <stdio.h>

#ifndef NDEBUG
// Include deubg information by default if assertions are enabled (meaning debug)
#ifndef BASIC_LOGGER_MIN_SEVERITY
#define BASIC_LOGGER_MIN_SEVERITY LEVEL_DEBUG
#endif

#ifndef BASIC_LOGGER_DEFAULT_LEVEL
#define BASIC_LOGGER_DEFAULT_LEVEL BASIC_LOGGER_MIN_SEVERITY
#endif

#ifndef BASIC_LOGGER_USE_COLOR_CODES
#define BASIC_LOGGER_USE_COLOR_CODES 1
#endif

#ifndef BASIC_LOGGER_PRINT_SOURCE_LOCATION
#define BASIC_LOGGER_PRINT_SOURCE_LOCATION 1
#endif

#else
// Disable debug information if assertions are disabled (meaning release)
#ifndef BASIC_LOGGER_MIN_SEVERITY
#define BASIC_LOGGER_MIN_SEVERITY LEVEL_INFO
#endif

#ifndef BASIC_LOGGER_DEFAULT_LEVEL
#define BASIC_LOGGER_DEFAULT_LEVEL BASIC_LOGGER_MIN_SEVERITY
#endif

#ifndef BASIC_LOGGER_USE_COLOR_CODES
#define BASIC_LOGGER_USE_COLOR_CODES 0
#endif

#ifndef BASIC_LOGGER_PRINT_SOURCE_LOCATION
#define BASIC_LOGGER_PRINT_SOURCE_LOCATION 0
#endif
#endif

#define LEVEL_DEBUG 1
#define LEVEL_INFO  2
#define LEVEL_WARN  3
#define LEVEL_ERROR 4
#define LEVEL_FAULT 5   // NOTE: Special level for safety fault messages
#define LEVEL_FATAL 6
#define LEVEL_NO_LOGGING 6

#if BASIC_LOGGER_DEFAULT_LEVEL < BASIC_LOGGER_MIN_SEVERITY
#error Default logging level is below minimum severity
#endif

typedef void (*custom_logger_cb_t)(void* args, const int log_level, const char * unit, const char * filename, const int line, const char * const function, const char * const fmt, va_list argp);
extern void basic_logger_set_global_log_level(int log_level);
extern void basic_logger_set_custom_logger_callback(custom_logger_cb_t callback, void* args);
extern void basic_logger_log_common(const int log_level, const int local_log_level, const char * unit, const char * filename, const int line, const char * const function, const char * const fmt, ...);


// Logging Unit Definitions
// These can be undefined and redefined to change local log settings
#define LOGGING_UNIT_NAME "No Unit"
#define LOGGING_UNIT_LOCAL_LEVEL BASIC_LOGGER_DEFAULT_LEVEL


// Logging function definitions
#if BASIC_LOGGER_MIN_SEVERITY > LEVEL_DEBUG
#define LOG_DEBUG(...) do {} while(0);
#else
#if BASIC_LOGGER_PRINT_SOURCE_LOCATION
#define LOG_DEBUG(...) do {basic_logger_log_common(LEVEL_DEBUG, LOGGING_UNIT_LOCAL_LEVEL, LOGGING_UNIT_NAME, __FILE__, __LINE__, __func__, __VA_ARGS__);} while(0);
#else
#define LOG_DEBUG(...) do {basic_logger_log_common(LEVEL_DEBUG, LOGGING_UNIT_LOCAL_LEVEL, LOGGING_UNIT_NAME, "", 0, "", __VA_ARGS__);} while(0);
#endif
#endif

#if BASIC_LOGGER_MIN_SEVERITY > LEVEL_INFO
#define LOG_INFO(...) do {} while(0);
#else
#if BASIC_LOGGER_PRINT_SOURCE_LOCATION
#define LOG_INFO(...) do {basic_logger_log_common(LEVEL_INFO, LOGGING_UNIT_LOCAL_LEVEL, LOGGING_UNIT_NAME, __FILE__, __LINE__, __func__, __VA_ARGS__);} while(0);
#else
#define LOG_INFO(...) do {basic_logger_log_common(LEVEL_INFO, LOGGING_UNIT_LOCAL_LEVEL, LOGGING_UNIT_NAME, "", 0, "", __VA_ARGS__);} while(0);
#endif
#endif

#if BASIC_LOGGER_MIN_SEVERITY > LEVEL_WARN
#define LOG_WARN(...) do {} while(0);
#else
#if BASIC_LOGGER_PRINT_SOURCE_LOCATION
#define LOG_WARN(...) do {basic_logger_log_common(LEVEL_WARN, LOGGING_UNIT_LOCAL_LEVEL, LOGGING_UNIT_NAME, __FILE__, __LINE__, __func__, __VA_ARGS__);} while(0);
#else
#define LOG_WARN(...) do {basic_logger_log_common(LEVEL_WARN, LOGGING_UNIT_LOCAL_LEVEL, LOGGING_UNIT_NAME, "", 0, "", __VA_ARGS__);} while(0);
#endif
#endif

#if BASIC_LOGGER_MIN_SEVERITY > LEVEL_ERROR
#define LOG_ERROR(...) do {} while(0);
#else
#if BASIC_LOGGER_PRINT_SOURCE_LOCATION
#define LOG_ERROR(...) do {basic_logger_log_common(LEVEL_ERROR, LOGGING_UNIT_LOCAL_LEVEL, LOGGING_UNIT_NAME, __FILE__, __LINE__, __func__, __VA_ARGS__);} while(0);
#else
#define LOG_ERROR(...) do {basic_logger_log_common(LEVEL_ERROR, LOGGING_UNIT_LOCAL_LEVEL, LOGGING_UNIT_NAME, "", 0, "", __VA_ARGS__);} while(0);
#endif
#endif

#if BASIC_LOGGER_MIN_SEVERITY > LEVEL_FAULT
#define LOG_FAULT(...) do {} while(0);
#else
#if BASIC_LOGGER_PRINT_SOURCE_LOCATION
#define LOG_FAULT(...) do {basic_logger_log_common(LEVEL_FAULT, LOGGING_UNIT_LOCAL_LEVEL, LOGGING_UNIT_NAME, __FILE__, __LINE__, __func__, __VA_ARGS__);} while(0);
#else
#define LOG_FAULT(...) do {basic_logger_log_common(LEVEL_FAULT, LOGGING_UNIT_LOCAL_LEVEL, LOGGING_UNIT_NAME, "", 0, "", __VA_ARGS__);} while(0);
#endif
#endif

#if BASIC_LOGGER_MIN_SEVERITY > LEVEL_FATAL
#define LOG_FATAL(...) do {} while(0);
#else
#if BASIC_LOGGER_PRINT_SOURCE_LOCATION
#define LOG_FATAL(...) do {basic_logger_log_common(LEVEL_FATAL, LOGGING_UNIT_LOCAL_LEVEL, LOGGING_UNIT_NAME, __FILE__, __LINE__, __func__, __VA_ARGS__);} while(0);
#else
#define LOG_FATAL(...) do {basic_logger_log_common(LEVEL_FATAL, LOGGING_UNIT_LOCAL_LEVEL, LOGGING_UNIT_NAME, "", 0, "", __VA_ARGS__);} while(0);
#endif
#endif

#endif
