#ifndef CORE1_H
#define CORE1_H

#include <stdbool.h>
#include <stdint.h>

void core1_init();
void core1_update_target_rpm(const int16_t *rpm);
void core1_set_raw_mode(bool raw_mode);
bool core1_get_current_rpm(int thruster_num, int16_t *rpm_out, int16_t *cmd_out);
bool core1_set_p_gain(int32_t value);
bool core1_set_i_gain(int32_t value);
bool core1_set_i_bound(int32_t value);
bool core1_set_hard_limit(int32_t value);
bool core1_set_min_command(int32_t value);
float core1_get_loop_rate(void);

#endif
