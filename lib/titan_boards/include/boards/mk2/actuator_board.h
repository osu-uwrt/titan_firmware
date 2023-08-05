#ifndef BOARDS__MK2__ACTUATOR_BOARD_H_
#define BOARDS__MK2__ACTUATOR_BOARD_H_

#include "boards/mk2/blocks/rp2040_can_block.h"

#ifdef UWRT_BOARD_DEFINED
#error Multiple board types defined
#endif
#define UWRT_BOARD_DEFINED

// This defines which CAN bus this board is connected into
// The CAN bus is defined in the corresponding robot definition files (rate, enable FD, etc.)
#define CAN_BUS_NAME INTERNAL_CAN
#define CAN_BUS_CLIENT_ID 5

// Claw
#define CLAW_ENABLE_PIN     0
#define CLAW_SLEEP_PIN      1
#define CLAW_PHASE_PIN      2
#define CLAW_MODE1_PIN      3
#define CLAW_FAULT_PIN      4
#define CLAW_MODE2_PIN      5
#define CLAW_PWM_PIN        9
#define CLAW_CHECK_PIN      10
#define CLAW_CURRENT_PIN    29

#define RGB_DATA_PIN        12

// Torpedos
#define TORP_SEL_2_PIN      6
#define TORP_SEL_1_PIN      7
#define TORP_DRAIN_PIN      8
#define TORP_ARM_PIN        11
#define COIL_3_PIN          25
#define COIL_2_PIN          26
#define COIL_1_PIN          27
#define TORP_CHARGE_LVL     28

// Dropper
#define DROPPER_2_PIN       23
#define DROPPER_1_PIN       24

#endif
