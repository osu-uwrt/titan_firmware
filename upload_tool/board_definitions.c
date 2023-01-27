#include <stdbool.h>
#include "uploader.h"

const struct rp2040_device_capabilities board_types[] = {
    {
        .board_type_name = "UNKNOWN",
        .supports_general_code = false,
        .supports_i2c_upload = false,
        .supports_i2c_proxy = false,
    },
    {
        .board_type_name = "pico",
        .supports_general_code = true,
        .supports_i2c_upload = false,
        .supports_i2c_proxy = false,
    },
    {
        .board_type_name = "rp2040_eval_board",
        .supports_general_code = true,
        .supports_i2c_upload = false,
        .supports_i2c_proxy = false,
    },
    {
        .board_type_name = "backplane_breakout_board",
        .supports_general_code = false,
        .supports_i2c_upload = false,
        .supports_i2c_proxy = true,
    },
    {
        .board_type_name = "backplane_breakout_board_eval_hat",
        .supports_general_code = false,
        .supports_i2c_upload = false,
        .supports_i2c_proxy = true,
    },
    {
        .board_type_name = "actuator_mk1_hat",
        .supports_general_code = false,
        .supports_i2c_upload = true,
        .supports_i2c_proxy = false,
    },
    {
        .board_type_name = "puddles_backplane",
        .supports_general_code = true,
        .supports_i2c_upload = false,
        .supports_i2c_proxy = false,
    },
    {
        .board_type_name = "jerry",
        .supports_general_code = false,
        .supports_i2c_proxy = false,
        .supports_i2c_upload = false
    },
    {
        .board_type_name = "mk2_esc_board",
        .supports_general_code = false,
        .supports_i2c_proxy = false,
        .supports_i2c_upload = false
    }
};
const struct rp2040_device_capabilities *unknown_board_type = &board_types[0];

const size_t num_board_types = sizeof(board_types) / sizeof(struct rp2040_device_capabilities);

// Note: Ensure that the board type name exists in boards or else it will fail during load
const struct rp2040_board_instance board_definitions[] = {
    {	// Robert Pafford's Pico
        .board_type_name = "pico",
        .serial = "E66038B713849D31",
    },
    {   // The first backplane breakout board, built with a pico
        .board_type_name = "backplane_breakout_board",
        .serial = "E660B440070E5E2A",
    },
    {   // Team pico in Arduino box
        .board_type_name = "pico",
        .serial = "E660583883731D2D",
    },
    {   // The backplane breakout board that is designed for puddles
        .board_type_name = "backplane_breakout_board",
        .serial = "E660B440075F7B26"
    },
    {
        .board_type_name = "actuator_mk1_hat",
        .serial = "E460CCB01B3F532C"
    },
    {
        .board_type_name = "actuator_mk1_hat",
        .serial = "E460CCB01B29352A"
    },
    {   // Team raspberry pi pico (4th one in the order box)
        .board_type_name = "pico",
        .serial = "E660B44007924D2A"
    },
    {   // First puddles backplane assembled
        .board_type_name = "puddles_backplane",
        .serial = "E462288893572D28"
    },
    {   // One of the first picos the team bought, taken during competition for use in the abomination of the 2022 Competition Actuator Board
        .board_type_name = "terry",
        .serial = "E660583883754F2C"
    },
    {   // Team raspberry pi pico (5th one in the order box)
        .board_type_name = "jerry",
        .serial = "E660B440076B4B2A"
    },
    {   // Mark 2 ESC Board Thrusters 1-4
        .board_type_name = "mk2_esc_board",
        .serial = "E6621455075D8323"
    }
};

const size_t num_board_definitions = sizeof(board_definitions) / sizeof(struct rp2040_board_instance);
