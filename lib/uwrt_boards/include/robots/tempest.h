#ifndef _ROBOT__TEMPEST_H
#define _ROBOT__TEMPEST_H

#ifdef UWRT_ROBOT_DEFINED
#error Multiple robot types defined
#endif
#define UWRT_ROBOT_DEFINED


#define ROBOT_NAMESPACE "tempest"

#define HW_USE_DSHOT 0
#define HW_USE_PWM   1

#define INTERNAL_CAN_ENABLE_FD  1
#define INTERNAL_CAN_RATE       1000000
#define INTERNAL_CAN_FD_RATE    5000000

#define EXTERNAL_CAN_ENABLE_FD  0
#define EXTERNAL_CAN_RATE       250000

#endif