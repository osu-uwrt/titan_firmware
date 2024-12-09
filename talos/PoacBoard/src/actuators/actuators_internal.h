#ifndef ACTUATORS_V2__ACTUATORS_INTERNAL_H
#define ACTUATORS_V2__ACTUATORS_INTERNAL_H

#include "actuators.h"

#include "driver/dynamixel.h"

#include <stdint.h>

/**
 * @brief Margin (in each direction) from the target position where the move is considered complete
 * Used to determine when the move has successfully stopped in the target position
 *
 * 64: ~5.625 deg in each direction / 11.25 deg total
 */
#define TARGET_POSITION_MARGIN 96

// DXL Actuator Base Definitions

typedef struct dxl_actuator_base_state dxlact_state_t;

typedef void (*dxlact_idle_position_handler_t)(dxlact_state_t *state, int32_t current_position);
typedef bool (*dxlact_move_done_handler_t)(dxlact_state_t *state, int32_t *next_target);

typedef struct dxl_actuator_base_state {
    dynamixel_id id;
    uint32_t max_move_time_ms;

    // Handlers
    dxlact_idle_position_handler_t idle_handler;
    dxlact_move_done_handler_t done_handler;

    // Internal state tracking, safe to modify in interrupts
    volatile bool connected;
    volatile bool move_active;
    volatile bool enabled;
    volatile bool homed;
    volatile bool hardware_err;

    // Only valid when move_active is true
    int32_t target_position;
    absolute_time_t move_timeout;
} dxlact_state_t;

void dxlact_base_initialize(dxlact_state_t *state, dynamixel_id id, dxlact_idle_position_handler_t idle_handler,
                            dxlact_move_done_handler_t done_handler, uint32_t max_move_time_ms);
bool dxlact_base_set_target(dxlact_state_t *state, const char **errMsgOut, int32_t target_position);
void dxlact_base_report_state(dxlact_state_t *state, bool torque_enabled, bool moving, int32_t target_position,
                              int32_t current_position, uint8_t hardware_err_status);
void dxlact_base_report_connect(dxlact_state_t *state);
void dxlact_base_report_disconnect(dxlact_state_t *state);
bool dxlact_base_arm(dxlact_state_t *state, const char **errMsgOut);
void dxlact_base_safety_disable(dxlact_state_t *state);
bool dxlact_base_get_busy(dxlact_state_t *state);

void torpedo_marker_initialize(dxlact_state_t *state, dynamixel_id id);
void torpedo_marker_reset_count(void);
void claw_initialize(dxlact_state_t *state, dynamixel_id id);

#endif
