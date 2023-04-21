#ifndef DYNAMIXEL_H
#define DYNAMIXEL_H

#include <stdint.h>
#include <stdlib.h>

enum dynamixel_error { 
    DYNAMIXEL_ERROR_NONE = 0,
    /* An internal error in the driver */
    DYNAMIXEL_DRIVER_ERROR,
    /* An internal error in the TTL driver */
    DYNAMIXEL_TTL_ERROR,
    /* This is triggered when a request was invalid */
    DYNAMIXEL_REQUEST_ERROR,
    /* This is triggered when a packet is not properly parsed. */
    DYNAMIXEL_PACKET_ERROR,
    /* This is flagged when a */
    HARDWARE_ERROR,
};

typedef uint8_t dynamixel_id;

typedef void (*dynamixel_error_cb)(enum dynamixel_error error_code);

void dynamixel_init(dynamixel_id *id_list, size_t id_cnt, dynamixel_error_cb error_cb);

void dynamixel_set_id();

void dynamixel_ping();

void dynamixel_reboot();

void dynamixel_factory_reset();

/* How control servo (?) */

#endif