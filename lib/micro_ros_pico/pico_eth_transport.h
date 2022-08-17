#ifndef MICRO_ROS_PICOSDK
#define MICRO_ROS_PICOSDK

#include <stdio.h>
#include <stdint.h>

#include <uxr/client/profile/transport/custom/custom_transport.h>

void serial_init_early(void);
void pico_eth_transport_init(int sock_num, uint32_t target_ip, uint16_t target_port);

//bool pico_eth_transport_open(struct uxrCustomTransport * transport);
//bool pico_eth_transport_close(struct uxrCustomTransport * transport);
//size_t pico_eth_transport_write(struct uxrCustomTransport* transport, const uint8_t * buf, size_t len, uint8_t * err);
//size_t pico_eth_transport_read(struct uxrCustomTransport* transport, uint8_t* buf, size_t len, int timeout, uint8_t* err);

#endif //MICRO_ROS_PICOSDK
