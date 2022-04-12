#include "basic_logger/logging.h"

#include "pico/time.h"

static int dynamic_global_log_level = BASIC_LOGGER_DEFAULT_LEVEL;
static custom_logger_cb_t custom_cb = NULL;
static void *custom_cb_args = NULL;

#if BASIC_LOGGER_USE_COLOR_CODES
#define COLOR_DEBUG_NUM 4
#define COLOR_INFO_NUM  7
#define COLOR_WARN_NUM  3
#define COLOR_ERROR_NUM 1
#define COLOR_FAULT_NUM 1
#define COLOR_FATAL_NUM 1

#define COLOR_BRIGHT 9
#define COLOR_DARK   3

#define xstr(s) str(s)
#define str(s) #s

#define COLOR_TAG_DEBUG "\033[1;" xstr(COLOR_DARK) xstr(COLOR_DEBUG_NUM) "m"
#define COLOR_TAG_INFO  "\033[1;" xstr(COLOR_DARK) xstr(COLOR_INFO_NUM) "m"
#define COLOR_TAG_WARN  "\033[1;" xstr(COLOR_DARK) xstr(COLOR_WARN_NUM) "m"
#define COLOR_TAG_ERROR "\033[1;" xstr(COLOR_DARK) xstr(COLOR_ERROR_NUM) "m"
#define COLOR_TAG_FAULT "\033[1;38:5:166m"
#define COLOR_TAG_FATAL "\033[1;" xstr(COLOR_DARK) xstr(COLOR_FATAL_NUM) "m"

#define COLOR_MSG_DEBUG "\033[0;" xstr(COLOR_BRIGHT) xstr(COLOR_DEBUG_NUM) "m"
#define COLOR_MSG_INFO  "\033[0;" xstr(COLOR_BRIGHT) xstr(COLOR_INFO_NUM) "m"
#define COLOR_MSG_WARN  "\033[0;" xstr(COLOR_BRIGHT) xstr(COLOR_WARN_NUM) "m"
#define COLOR_MSG_ERROR "\033[0;" xstr(COLOR_BRIGHT) xstr(COLOR_ERROR_NUM) "m"
#define COLOR_MSG_FAULT "\033[0;38:5:202m"
#define COLOR_MSG_FATAL "\033[0;" xstr(COLOR_BRIGHT) xstr(COLOR_FATAL_NUM) "m"

#define TAG_COLOR_STRING(level) ( \
    (level == LEVEL_DEBUG) ? COLOR_TAG_DEBUG : \
    (level == LEVEL_INFO)  ? COLOR_TAG_INFO : \
    (level == LEVEL_WARN)  ? COLOR_TAG_WARN : \
    (level == LEVEL_ERROR) ? COLOR_TAG_ERROR : \
    (level == LEVEL_FAULT) ? COLOR_TAG_FAULT : \
    (level == LEVEL_FATAL) ? COLOR_TAG_FATAL : \
    "" \
)

#define MSG_COLOR_STRING(level) ( \
    (level == LEVEL_DEBUG) ? COLOR_MSG_DEBUG : \
    (level == LEVEL_INFO)  ? COLOR_MSG_INFO : \
    (level == LEVEL_WARN)  ? COLOR_MSG_WARN : \
    (level == LEVEL_ERROR) ? COLOR_MSG_ERROR : \
    (level == LEVEL_FAULT) ? COLOR_MSG_FAULT : \
    (level == LEVEL_FATAL) ? COLOR_MSG_FATAL : \
    "" \
)

#define CLEAR_COLOR_STRING "\033[0m"

#else

#define TAG_COLOR_STRING(level) ""
#define MSG_COLOR_STRING(level) ""
#define CLEAR_COLOR_STRING ""

#endif

#define GET_LEVEL_STRING(level) ( \
    (level == LEVEL_DEBUG) ? "DEBUG" : \
    (level == LEVEL_INFO)  ? "INFO" : \
    (level == LEVEL_WARN)  ? "WARN" : \
    (level == LEVEL_ERROR) ? "ERROR" : \
    (level == LEVEL_FAULT) ? "FAULT" : \
    (level == LEVEL_FATAL) ? "FATAL" : \
    "INVALID_LEVEL" \
)

void basic_logger_set_global_log_level(int log_level) {
    dynamic_global_log_level = log_level;
}

void basic_logger_set_custom_logger_callback(custom_logger_cb_t callback, void* args) {    
    if (custom_cb != NULL && custom_cb != callback) {
        LOG_WARN("Custom log callback replacing current callback");
    }
    custom_cb = callback;
    custom_cb_args = args;
}

void basic_logger_remove_custom_logger_callback(custom_logger_cb_t callback) {
    if (custom_cb != callback) {
        LOG_WARN("Not removing custom logger callback because it is not the actively registered callback");
    } else {
        custom_cb = NULL;
        custom_cb_args = NULL;
    }
}

void basic_logger_log_common(const int log_level, const int local_log_level, const char * unit, const char * filename, const int line, const char * const function, const char * const fmt, ...) {
    if (log_level >= dynamic_global_log_level || log_level >= local_log_level) {
        double uptime_seconds = to_us_since_boot(get_absolute_time()) / 1E6;

#if BASIC_LOGGER_USE_COLOR_CODES
        printf(TAG_COLOR_STRING(log_level));
#endif


#if BASIC_LOGGER_PRINT_SOURCE_LOCATION
        printf("[%s] [%.6f] [%s] (%s:%d): ", GET_LEVEL_STRING(log_level), uptime_seconds, unit, filename, line);
#else  
        printf("[%s] [%.6lf] [%s]: ", GET_LEVEL_STRING(log_level), uptime_seconds, unit);
#endif
        printf(MSG_COLOR_STRING(log_level));

        va_list args;
        va_start (args, fmt);
        vprintf(fmt, args);
        printf(CLEAR_COLOR_STRING "\n");
        
        if (custom_cb) {
            custom_cb(custom_cb_args, log_level, unit, filename, line, function, fmt, args);
        }
        va_end (args);
    }
}
