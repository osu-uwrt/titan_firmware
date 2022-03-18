#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "hardware/watchdog.h"
#include "hardware/timer.h"
#include "hardware/clocks.h"
#include "hardware/gpio.h"
#include <string.h>
#include "hardware/pwm.h"
#include "hardware/irq.h"
//Globals are defined here


//Type of variable definitions
typedef unsigned int uint;
typedef unsigned short uint16;
typedef unsigned long uint32;
typedef unsigned char uint8;

//I2C port
#define I2C_PORT i2c0

//GPIO PORT NUMBERS
#define I2C_SDA 0
#define I2C_SCL 1
#define LED_1 4
#define LED_2 5
#define COIL_3_GATE 11
#define COIL_5_GATE 12
#define COIL_S_GATE 13
#define COIL_2_GATE 14
#define COIL_1_GATE 15
#define COIL_4_GATE 16
#define TORPEDO1_GATE 17
#define TORPEDO2_GATE 18
#define ARM_TORPEDO1 19
#define DROPPER1_GATE 20
#define DROPPER2_GATE 21
#define PWM_CLAW 22
#define PWM_DRIVER1 23
#define PWM_DRIVER2 24

//Globals
#define OUT 1

//#define HIGHPRIOR 0;
//#define ENABLE TRUE;



//Function headers
void init_pwm(uint16 top);
void init_gpio();
void init_i2c();
void init_irq();
void init_new_claw();
void init_dropper();
void drop_marker(uint);
void set_new_claw(int, uint);
void set_old_claw(int, uint);
void old_claw_setup(uint16, uint32);
void hard_stop();