#ifndef HBRIDGE_BOARD_CLAW_H
#define HBRIDGE_BOARD_CLAW_H
#include "hiwonder_driver.h"
#include "servo_position.h"

typedef struct {
    servo_t servo;
    servo_position_t position;

    int32_t target_position;

    bool moving;
} claw_t;

void claw_init(void);


bool claw_set_position(int32_t position);
int32_t claw_get_position(void);
bool claw_get_raw_position(int16_t *position_out);
bool claw_get_voltage(uint16_t *voltage_out);

bool claw_tare(void);
bool claw_is_referenced(void);
servo_t *claw_get_servo(void);
void claw_notify_disable(void);

bool claw_open(void);
bool claw_close(void);

void claw_stop(void);
void claw_update(void);

#endif  // HBRIDGE_BOARD_CLAW_H
