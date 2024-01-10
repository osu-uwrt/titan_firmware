#ifndef THRUSTERCONTROLLER_HH
#define THRUSTERCONTROLLER_HH

#include <stdbool.h>
#include <stdint.h>
#include "pico/time.h"

/**
 * @brief The length of the average buffer for the controller
 */
#define CONTROL_AVG_LENGTH 100

typedef struct thruster_controller_state {
    // === gains and bounds ===

    // P gains have been mutliplied by 1,000,000 - don't set to more that 1,000,000
    int32_t Pgain;
    // I gain has been mutliplied by 1,000,000 - don't set to more that 1,000
    int32_t Igain;
    int32_t Ibound;

    // the max dshot limit this controller will send to the thruster.
    int16_t hardLimit;

    // The minimum dshot command that can be sent to the ESC
    int16_t minCommand;

    // Set to true if the thruster is inverted
    bool inverted;

    // === State ===
    // sum of error for I
    int64_t sumOfError;

    // the last target at which the I was reset
    int32_t lastIReset;

    // for the rolling averge rpm
    int32_t avgBuffer[CONTROL_AVG_LENGTH];
    int32_t valueToSet;
    int32_t rollingAvg;

    // has the rolling rpm buffer been filled yet
    bool avgBufferFilled;

    // Thruster Direction Change Timeout
    absolute_time_t directionChangeTimeout;

} thruster_controller_state_t;

static inline void thruster_controller_zero(thruster_controller_state_t *state) {
    state->sumOfError = 0;
    state->valueToSet = 0;
    state->rollingAvg = 0;
    state->avgBufferFilled = false;
    state->lastIReset = 0;
}

/**
 * @brief Ticks the dshot thruster controller
 *
 * @param state Holds controller state
 * @param targetRPM Holds the target RPM for the controller to achieve
 * @param currentRPM Holds the current RPM of the thruster
 * @param deltaTime The time between two ticks
 * @return int16_t The dshot command to send (between -hardLimit and hardLimit)
 */
int16_t thruster_controller_tick(thruster_controller_state_t *state, int32_t targetRPM, int32_t CurrentRPM,
                                 int32_t deltaTime);

#endif
