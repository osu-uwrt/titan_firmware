#ifndef THRUSTERCONTROLLER_HH
#define THRUSTERCONTROLLER_HH

#include <stdint.h>

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

    // === State ===
    // sum of error for I
    int64_t sumOfError;

    // the last target at which the I was reset
    int32_t lastIReset;

    // for the rolling averge rpm
    int32_t avgBuffer[100];
    int32_t valueToSet;
    int32_t rollingAvg;

    // has the rolling rpm buffer been filled yet
    uint8_t avgBufferFilled;

} thruster_controller_state_t;

/**
 * @brief Initialize the controller state variable
 *
 * @param state Pointer to controller state to initialize
 */
static inline void thruster_controller_init_defaults(thruster_controller_state_t *state) {
    state->Pgain = 100000;
    state->Igain = 1000;
    state->Ibound = 300;
    state->hardLimit = 725;
    state->minCommand = 0;  // TODO: Set these

    state->sumOfError = 0;
    state->lastIReset = 0;

    state->valueToSet = 0;
    state->rollingAvg = 0;

    state->avgBufferFilled = 0;
}

static inline void thruster_controller_zero(thruster_controller_state_t *state) {
    state->sumOfError = 0;
}

/**
 * @brief Ticks the dshot thruster controller
 *
 * @param state Holds controller state
 * @param targetRPM Holds the target RPM for the controller to achieve
 * @param currentRPM Holds the current RPM of the thruster
 * @param deltaTime The time between two ticks
 * @param inverted if the thruster blades are in the backwards direction
 * @return int16_t The dshot command to send (between -hardLimit and hardLimit)
 */
int16_t thruster_controller_tick(thruster_controller_state_t *state, int32_t targetRPM, int32_t CurrentRPM,
                                 int32_t deltaTime, uint8_t inverted);

/**
 *
 * @brief Uodates the rolling average being used as a control signal
 *
 */
void thruster_controller_process_rolling_average(thruster_controller_state_t *state, int32_t currentRPM);

#endif
