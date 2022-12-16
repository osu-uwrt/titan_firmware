#ifndef MICRO_ROS_PICOSDK
#define MICRO_ROS_PICOSDK

#include <stdio.h>
#include <stdint.h>

#include "hardware/spi.h"

void pico_can_transport_init(uint bitrate,
                             uint client_id,
                             spi_inst_t* spi_channel,
                             uint8_t cs_pin,
                             uint8_t mosi_pin,
                             uint8_t miso_pin,
                             uint8_t sck_pin,
                             uint32_t spi_clock,
                             uint8_t int_pin);

#endif //MICRO_ROS_PICOSDK
