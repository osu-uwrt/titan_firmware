#include "pico/time.h"
#include "titan/logger.h"

#include <rmw_microros/rmw_microros.h>

#include <time.h>

#undef LOGGING_UNIT_NAME
#define LOGGING_UNIT_NAME "micro_ros"

// Micro ROS required functions
void usleep(uint64_t us) {
    sleep_us(us);
}

int clock_gettime(__unused clockid_t unused, struct timespec *tp) {
    uint64_t m = time_us_64();
    tp->tv_sec = m / 1000000;
    tp->tv_nsec = (m % 1000000) * 1000;
    return 0;
}

// ========================================
// RMW Error Handling Code
// ========================================

const char *const entity_lookup_table[] = {
    "RMW_UROS_ERROR_ON_UNKNOWN",      "RMW_UROS_ERROR_ON_NODE",
    "RMW_UROS_ERROR_ON_SERVICE",      "RMW_UROS_ERROR_ON_CLIENT",
    "RMW_UROS_ERROR_ON_SUBSCRIPTION", "RMW_UROS_ERROR_ON_PUBLISHER",
    "RMW_UROS_ERROR_ON_GRAPH",        "RMW_UROS_ERROR_ON_GUARD_CONDITION",
    "RMW_UROS_ERROR_ON_TOPIC",
};
const char *const source_lookup_table[] = {
    "RMW_UROS_ERROR_ENTITY_CREATION", "RMW_UROS_ERROR_ENTITY_DESTRUCTION",    "RMW_UROS_ERROR_CHECK",
    "RMW_UROS_ERROR_NOT_IMPLEMENTED", "RMW_UROS_ERROR_MIDDLEWARE_ALLOCATION",
};

#define lookup_string_enum(value, list) ((value < sizeof(list) / sizeof(*list)) ? list[value] : "Out-of-Bounds")
#define lookup_entity_enum(value) lookup_string_enum(value, entity_lookup_table)
#define lookup_source_enum(value) lookup_string_enum(value, source_lookup_table)

void rmw_error_cb(__unused const rmw_uros_error_entity_type_t entity, __unused const rmw_uros_error_source_t source,
                  __unused const rmw_uros_error_context_t context, __unused const char *file, __unused const int line) {
    LOG_DEBUG("RMW UROS Error:\n\tEntity: %s\n\tSource: %s\n\tDesc: %s\n\tLocation: %s:%d", lookup_entity_enum(entity),
              lookup_source_enum(source), context.description, file, line);
}

void micro_ros_init_error_handling(void) {
    rmw_uros_set_error_handling_callback(rmw_error_cb);
}
