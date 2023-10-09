#include "ThrusterController.h"

#define SPLINES 10

// calc ff dshot value
static int32_t thruster_controller_feed_forward(int32_t targetRPM) {
    // use matlab script in this folder to generate bounds and coefficents from points

    // clang-format off
    // where to switch from one coefficent to the next
    //these are rpm values
    int32_t ffBounds[SPLINES - 1] = {56,113,168,184,199,227,252,285,312,353,395,427,470,495,563,582,633,696,753,836,898,978,1059,1160,1260,1361,1451,1594,1685,1806,1947,2076,2210,2344,2475,2648,2851};

    // x^3 x^2 x 1  - these coefficents are multiplied by a million
    int32_t ffCoefficents[SPLINES][6] =  {{-15,2028,-32817,20142148},{4,-1201,148031,16766315},{4,-1320,161426,16261782},{78,-38395,6390043,-332540780},{-217,124642,-23608837,1507390532},{101,-65543,14238038,-1003118884},{-73,53199,-12716429,1036435820},{36,-29301,8073732,-709937744},{-27,24258,-7190779,740190787},{3,-3231,1385954,-151789404},{20,-21769,7929727,-921773375},{-52,64288,-26062630,3553886942},{63,-83343,36975775,-5418579292},{-155,223585,-107280415,17181557079},{73,-114929,60283742,-10466528860},{-269,462558,-264841402,50548623285},{53,-98789,61862718,-12831976071},{-5,10729,-7462316,1795606153},{-6,12157,-8456017,2026144716},{6,-14817,11855139,-3071955366},{-7,19215,-16595045,4856162542},{3,-9489,9180910,-2859440060},{-2,7559,-7492552,2576108644},{2,-6567,7467612,-2704829143},{-2,6094,-7218975,2973984539},{3,-12521,16235344,-6876829576},{-7,29329,-40722062,18962846934},{6,-26042,39620826,-19896329941},{-7,36723,-60425996,33261881319},{2,-10015,18326912,-10971001838},{1,-3055,5756802,-3403795700},{0,2491,-5040142,3603421347},{0,1177,-2312817,1716111837},{1,-5378,12172645,-8954844668},{-2,12895,-30658685,24510701233},{0,-3406,9686524,-8774096132},{1,-9419,25609120,-22828441399},{0,-439,7878,1501272074}};

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
