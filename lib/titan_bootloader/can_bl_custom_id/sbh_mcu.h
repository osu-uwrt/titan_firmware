#ifndef CAN_BL_CUSTOM_ID__SBH_MCU_H
#define CAN_BL_CUSTOM_ID__SBH_MCU_H

#include "hardware/flash.h"
#include "hardware/regs/addressmap.h"

#include <stdint.h>

// Determine client ID by the board detect pin
static bool bl_board_get_client_id(int *client_id) {
    uint8_t data[256];
    flash_read(0x1FF000, data, sizeof(data));
    if (data[0] == 0xFF || data[0] == 0) {
        return false;
    }

    *client_id = data[0];
    return true;
}

#endif
