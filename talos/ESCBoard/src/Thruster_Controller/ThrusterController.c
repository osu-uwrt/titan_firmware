#include "ThrusterController.h"

#define SPLINES 10

// calc ff dshot value
static int32_t thruster_controller_feed_forward(int32_t targetRPM) {
    // use matlab script in this folder to generate bounds and coefficents from points

    // clang-format off
    // where to switch from one coefficent to the next
    //these are rpm values
    int32_t ffBounds[SPLINES - 1] = {0,0,0,0,0,0,0,0,0};

    // x^3 x^2 x 1  - these coefficents are multiplied by a million
    int32_t ffCoefficents[SPLINES][6] =  {{0,0,0,0},
                                {0,0,0,0},
                                {0,0,0,0},
                                {0,0,0,0},
                                {0,0,0,0},
                                {0,0,0,0},
                                {0,0,0,0},
                                {0,0,0,0},
                                {0,0,0,0},
                                {0,0,0,0}};
    // clang-format on

    int8_t splineNum;
    for (splineNum = 0; splineNum < (SPLINES - 1); splineNum++) {
        // find the correct spline coefficents
        if (targetRPM < ffBounds[splineNum]) {
            break;
        }
    }

    // evaluate spline
    int32_t value = ffCoefficents[splineNum][0] * targetRPM * targetRPM * targetRPM +
                    ffCoefficents[splineNum][1] * targetRPM * targetRPM + ffCoefficents[splineNum][2] * targetRPM +
                    ffCoefficents[splineNum][3];

    // divide by 1000000 to account for scaled consts
    return value / 1000000;
}

// call this each cycle
int16_t thruster_controller_tick(thruster_controller_state_t *state, int32_t targetRPM, int32_t currentRPM,
                                 int32_t deltaTime) {
    // rpm error - this value should be practically less than 10,000
    int32_t currentError = targetRPM - currentRPM;

    // limit max error to ensure not overrunning int value
    if (currentError > 2100000000 / state->Pgain) {
        currentError = 2100000000 / state->Pgain;
    }
    else if (currentError < -2100000000 / state->Pgain) {
        currentError = -2100000000 / state->Pgain;
    }

    // calculate ff control
    int32_t ffControl = thruster_controller_feed_forward(targetRPM);

    // calculate p control - divide by 1000000 is scaling for const
    int32_t pControl = (currentError * state->Pgain) / 1000000;

    // calculate I
    int32_t iControl = 0;

    if (currentError < state->Ibound && currentError > -state->Ibound) {
        // only if within bound

        // add current to sum of error
        state->sumOfError += currentError * deltaTime;

        // check to ensure int value is not overrun - very possible when tuning gains
        if (state->sumOfError > 2100000000 / state->Igain) {
            state->sumOfError = 2100000000 / state->Igain;
        }
        else if (state->sumOfError < -2100000000 / state->Igain) {
            state->sumOfError = -2100000000 / state->Igain;
        }

        // divide by 1000000 is scaling for const
        iControl = (state->sumOfError * state->Igain) / 1000000;
    }
    else {
        // reset error sum (I) as the control has slipped bound
        state->sumOfError = 0;
    }

    // calculate total control
    int32_t dShotControl = iControl + pControl + ffControl;

    // enforce hard limit
    if (dShotControl > state->hardLimit) {
        dShotControl = state->hardLimit;
    }
    else if (dShotControl < -state->hardLimit) {
        dShotControl = -state->hardLimit;
    }

    if (dShotControl > 0 && dShotControl < state->minCommand) {
        dShotControl = 0;
    }

    if (dShotControl < 0 && dShotControl > -state->minCommand) {
        dShotControl = 0;
    }

    // Safe to directly cast because hard limit is a 16-bit int, which will restrict dShotControl
    return (int16_t) dShotControl;
}
