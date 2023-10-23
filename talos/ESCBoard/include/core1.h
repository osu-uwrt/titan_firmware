#ifndef CORE1_H
#define CORE1_H

#include "dshot.h"

#include <stdbool.h>
#include <stdint.h>

// Definitions for disabled flags
#define CORE1_DISABLED_SAFETY_KILLED 0
#define CORE1_DISABLED_ESC_BOARD_OFF 1
#define CORE1_DISABLED_ESC_POWERON_DELAY 2
#define CORE1_DISABLED_TGT_RPM_STALE 3
#define CORE1_DISABLED_TGT_RPM_STALE_PENALTY 4
#define CORE1_DISABLED_CONTROLLER_UNCONFIGURED 5

/**
 * @brief Core 1 Telemetry Data
 *
 * See core1_telem_shared_mem for documentation on this.
 */
struct core1_telem {
    uint32_t time_delta_us;
    uint16_t controller_tick_cnt;
    uint8_t disabled_flags;
    struct {
        int16_t cmd;
        int16_t rpm;
        bool rpm_valid;
        bool thruster_ready;
        uint16_t ticks_missed;
        uint16_t ticks_offline;
    } thruster[NUM_THRUSTERS];
};

// Exported functions
void core1_init(uint8_t board_id);
void core1_update_target_rpm(const int16_t *rpm);
void core1_get_telem(struct core1_telem *telem_out);

void core1_set_raw_mode(bool raw_mode);
bool core1_set_thruster_inverted_mask(int32_t value);
bool core1_set_p_gain(int32_t value);
bool core1_set_i_gain(int32_t value);
bool core1_set_i_bound(int32_t value);
bool core1_set_hard_limit(int32_t value);
bool core1_set_min_command(int32_t value);

#endif
