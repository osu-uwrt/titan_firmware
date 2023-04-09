#ifndef CAN_BL_CUSTOM_ID__MK2_ESC_BOARD_H
#define CAN_BL_CUSTOM_ID__MK2_ESC_BOARD_H

#include "hardware/gpio.h"
#include "hardware/timer.h"

// Determine client ID by the board detect pin
static bool bl_board_get_client_id(int *client_id) {
    gpio_init(BOARD_DET_PIN);
    gpio_pull_down(BOARD_DET_PIN);
    busy_wait_us(5);    // Give time for pull up to settle

    int esc_board_num = (gpio_get(BOARD_DET_PIN) ? 1 : 0);
    *client_id = (esc_board_num ? CAN_BUS_BOARD1_CLIENT_ID : CAN_BUS_BOARD0_CLIENT_ID);

    return true;
}


#endif