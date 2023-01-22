#ifndef MICRO_ROS_PICO__TRANSPORT_CAN_H
#define MICRO_ROS_PICO__TRANSPORT_CAN_H

#include <stdint.h>

#include "hardware/spi.h"

void transport_can_init(uint bitrate,
                             uint client_id,
                             spi_inst_t* spi_channel,
                             uint8_t cs_pin,
                             uint8_t mosi_pin,
                             uint8_t miso_pin,
                             uint8_t sck_pin,
                             uint32_t spi_clock,
                             uint8_t int_pin);

#endif //MICRO_ROS_PICO__TRANSPORT_CAN_H
