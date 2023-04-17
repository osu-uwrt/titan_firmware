
#include <stdint.h>

typedef void (*dynamixel_on_packet_complete)(uint8_t error);

enum DXLLibErrorCode dynamixel_send_ping(); 

void dynamixel_send_factory_reset();

void dynamixel_send_reboot();

void dynamixel_send_write_table();

void dynamixel_send_read_table();

void dynamixel_get_response();