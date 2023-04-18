#ifndef DYNAMIXEL_CONTROLS
#define DYNAMIXEL_CONTROLS

#define DYNAMIXEL_XL430 // TODO: move to board file
#ifdef DYNAMIXEL_XL430
// https://emanual.robotis.com/docs/en/dxl/x/xl430-w250/#control-table-of-eeprom-area

#define DYNAMIXEL_CTRL_TABLE_MODEL_ADDR 0 
#define DYNAMIXEL_CTRL_TABLE_MODEL_SIZE 2 

#define DYNAMIXEL_CTRL_TABLE_ID_ADDR 7
#define DYNAMIXEL_CTRL_TABLE_ID_SIZE 1

#else
#error Need to define dynamixel servo (XL430, ...)
#endif

#endif