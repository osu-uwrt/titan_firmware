#ifndef ACTUATORS_V2__ACTUATORS_INTERNAL_H
#define ACTUATORS_V2__ACTUATORS_INTERNAL_H

#include "dynamixel/dynamixel.h"

void torpedo_marker_initialize(dynamixel_id id);
bool torpedo_marker_arm(const char **errMsgOut);
void torpedo_marker_safety_disable(void);
void torpedo_marker_report_position(int32_t position);
void torpedo_marker_report_disconnect(void);

#endif