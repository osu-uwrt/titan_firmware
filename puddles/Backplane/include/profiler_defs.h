#ifndef PROFILER_DEFS_H
#define PROFILER_DEFS_H

#include <stdint.h>

enum profiler_ids {
    PROFILER_MAIN_LOOP = 0,
    PROFILER_BACKGROUND_TICK,
    PROFILER_EXECUTOR_TICK,
    PROFILER_ROS_TICK,
    PROFILER_ROS_CONFIG,

    PROFILER_DSHOT_TX_IRQ,
    PROFILER_DHSOT_TX_EVT,
    PROFILER_DSHOT_RX_IRQ0,
    PROFILER_DSHOT_RX_IRQ1,

    PROFILER_NUM_DEFS
};

static const char *const profiler_names[] = { "PROFILER_MAIN_LOOP",     "PROFILER_BACKGROUND_TICK",
                                              "PROFILER_EXECUTOR_TICK", "PROFILER_ROS_TICK",
                                              "PROFILER_ROS_CONFIG",

                                              "PROFILER_DSHOT_TX_IRQ",  "PROFILER_DSHOT_TX_EVT",
                                              "PROFILER_DSHOT_RX_IRQ0", "PROFILER_DSHOT_RX_IRQ1" };

#endif
