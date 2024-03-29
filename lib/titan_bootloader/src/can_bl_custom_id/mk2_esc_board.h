#ifndef CAN_BL_CUSTOM_ID__MK2_ESC_BOARD_H
#define CAN_BL_CUSTOM_ID__MK2_ESC_BOARD_H

#include "hardware/gpio.h"
#include "hardware/timer.h"
#include "titan/binary_info.h"

// Determine client ID by the board detect pin
static bool bl_board_get_client_id(int *client_id) {
    gpio_init(BOARD_DET_PIN);
    gpio_pull_down(BOARD_DET_PIN);
    busy_wait_us(5);  // Give time for pull up to settle

    // Define valid CAN IDs
    bi_decl_if_func_used(bi_client_id(CAN_BUS_BOARD0_CLIENT_ID));
    bi_decl_if_func_used(bi_client_id(CAN_BUS_BOARD1_CLIENT_ID));

    // Detect ID
    int esc_board_num = (gpio_get(BOARD_DET_PIN) ? 1 : 0);
    *client_id = (esc_board_num ? CAN_BUS_BOARD1_CLIENT_ID : CAN_BUS_BOARD0_CLIENT_ID);

    return true;
}

#endif
