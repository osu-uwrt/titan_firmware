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
        .board_type_name = "RP2040_eval_board",
        .supports_general_code = true,
        .supports_i2c_upload = false,
        .supports_i2c_proxy = false,
    },
    {
        .board_type_name = "backplane_breakout_board",
        .supports_general_code = false,
        .supports_i2c_upload = false,
        .supports_i2c_proxy = true,
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
};

const size_t num_board_definitions = sizeof(board_definitions) / sizeof(struct rp2040_board_instance);
