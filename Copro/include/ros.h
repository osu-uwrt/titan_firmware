#ifndef _ROS_H
#define _ROS_H

void ros_wait_for_connection(void);
void ros_start(const char* namespace);
void ros_spin_ms(long ms);

#endif