#ifndef ACTUATORS_V1__ACTUATORS_INTERNAL_H
#define ACTUATORS_V1__ACTUATORS_INTERNAL_H

void torpedo_initialize(void);
bool torpedo_arm(const char **errMsgOut);
void torpedo_safety_disable(void);

void dropper_initialize(void);
void dropper_safety_disable(void);

void claw_initialize(void);
void claw_safety_disable(void);

#endif