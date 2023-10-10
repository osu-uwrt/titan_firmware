#include "ThrusterController.h"

#define SPLINES 100

// calc ff dshot value
static int32_t thruster_controller_feed_forward(int32_t targetRPM) {
    // use matlab script in this folder to generate bounds and coefficents from points

    //dead zone
    if(targetRPM < 50 && targetRPM > -50){
        return 0;
    }

    // clang-format off
    // where to switch from one coefficent to the next
    //these are rpm values
    int32_t ffBounds[SPLINES - 1] = {-2949,-2767,-2620,-2482,-2334,-2141,-2018,-1890,-1763,-1618,-1495,-1402,-1279,-1173,-1091,-997,-917,-853,-753,-725,-642,-606,-555,-500,-476,-434,-384,-344,-318,-290,-253,-225,-198,-177,-163,-81,-45,0,4,59,113,166,181,199,222,263,292,313,348,377,427,465,488,556,570,642,686,729,827,911,955,1053,1138,1242,1347,1477,1582,1700,1818,1961,2108,2253,2404,2578,2748,2908};

    // x^3 x^2 x 1  - these coefficents are multiplied by a million
    int32_t ffCoefficents[SPLINES][6] = {{-22,-194256,-577103221,-571655015104},{4,35330,99946527,93884887767},{-2,-15871,-41725993,-36784400507},{0,1462,3685254,2874755865},{-1,-7298,-18056933,-15113280458},{2,13915,31456269,23407990885},{-3,-17339,-35458682,-24346979525},{1,6391,12426954,7864091944},{-1,-5212,-9502334,-5951359624},{1,3862,6496159,3450421178},{1,5916,9819284,5242693671},{-5,-19900,-28775836,-13990541361},{3,11148,14753205,6352030336},{0,569,1222126,583280578},{-3,-11374,-12786374,-4894042995},{2,7372,7665538,2543635579},{2,5776,6074442,2014861267},{-14,-39236,-35201541,-10601830647},{22,55275,45416208,12320482429},{-123,-272485,-201387310,-49627200581},{39,79171,53563711,11985962825},{-58,-107378,-66200773,-13643636692},{11,18060,9814429,1711434163},{25,41580,22867941,4126333784},{-110,-161024,-78433837,-12757295883},{40,52896,23392059,3399079710},{-9,-10885,-4288941,-605438277},{22,24468,9286466,1132213720},{-47,-46607,-15163264,-1671355228},{10,7598,2074006,155795365},{20,16298,4597069,399691422},{-33,-24063,-5614452,-461480139},{31,19173,4113662,268128393},{12,8090,1919232,123296023},{-289,-151500,-26328201,-1543302559},{124,50104,6533378,242176579},{-1170,-264319,-18934897,-445466848},{3499,366003,9429559,-20000000},{-55848,366003,9429559,-20000000},{2344,-332297,12222756,-23724262},{-653,198020,-19065899,591619278},{214,-95805,14136260,-658995387},{-256,138154,-24700840,1489990828},{-5,1850,-29879,1509548},{1,-1823,700967,-46969922},{13,-9364,2375046,-170851785},{12,-8931,2261283,-160878546},{-62,56027,-16706442,1685313331},{35,-34674,11682915,-1276642903},{-32,34931,-12539626,1533171875},{7,-9532,4222886,-573317111},{47,-60620,26037622,-3678281300},{-199,283263,-133868069,21107100832},{104,-161039,82951536,-14162221505},{-561,948666,-534044620,100187732756},{72,-134103,83133706,-17076149210},{-39,80187,-54440323,12364692951},{-6,12287,-7861012,1713557072},{4,-9361,7920363,-2121317067},{11,-27663,23056326,-6293797458},{-41,114653,-106594141,33076727778},{13,-41363,42401816,-14353651876},{-7,22902,-25269181,9398867998},{2,-8839,10851322,-4302842779},{-2,6880,-8671147,3779459471},{2,-8965,12671506,-5803391714},{-2,10989,-16799598,8706214810},{1,-6698,11180664,-6048709845},{-1,7638,-13190680,7761718376},{1,-4365,8631456,-5462495786},{0,-994,2020499,-1141133431},{0,2296,-4915122,3732296096},{0,1632,-3419582,2609145559},{0,-2716,7033422,-5767194828},{0,-2977,7705295,-6344557837},{-1,10554,-29476289,27713772605},{0,23,1147739,-1971118203}};
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
    //return value / 1000000;

    return 50;
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
