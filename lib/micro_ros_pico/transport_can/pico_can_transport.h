#ifndef MICRO_ROS_PICOSDK
#define MICRO_ROS_PICOSDK

#include <stdio.h>
#include <stdint.h>

void pico_can_transport_init(unsigned pio_num, unsigned bitrate, unsigned client_id, unsigned gpio_rx, unsigned gpio_tx, int gpio_term);

#endif //MICRO_ROS_PICOSDK
