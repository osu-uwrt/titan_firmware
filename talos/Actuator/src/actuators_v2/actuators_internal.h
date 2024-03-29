#ifndef ACTUATORS_V2__ACTUATORS_INTERNAL_H
#define ACTUATORS_V2__ACTUATORS_INTERNAL_H

#include "actuators.h"

#include "driver/dynamixel.h"

#include <stdint.h>

void torpedo_marker_initialize(dynamixel_id id);
bool torpedo_marker_arm(const char **errMsgOut);
void torpedo_marker_safety_disable(void);
void torpedo_marker_report_state(bool torque_enabled, bool moving, int32_t target_position, int32_t current_position,
                                 uint8_t hardware_err_status);
void torpedo_marker_report_connect(void);
void torpedo_marker_report_disconnect(void);
bool torpedo_marker_get_busy(void);

#endif
