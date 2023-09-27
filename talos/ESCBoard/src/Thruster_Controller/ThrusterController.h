#ifndef THRUSTERCONTROLLER_HH
#define THRUSTERCONTROLLER_HH

#include <cstdint>

class ThrusterController{
    private:
        //gains and bounds

        //P gains have been mutliplied by 1,000,000 - don't set to more that 1,000,000
        int32_t Pgain = 10000;
        // I gain has been mutliplied by 1,000,000 - don't set to more that 1,000
        int32_t Igain = 1;
        int32_t Ibound = 200;

        //the max dshot limit this controller will send to the thruster.
        int32_t hardLimit = 500;

    public:
        //sum of error for I
        int32_t sumOfError = 0;

        //calc ff dshot value
        int32_t feedForward(int32_t targetRPM);

        //call this each cycle
        int32_t control(int32_t targetRPM, int32_t currentRPM, int32_t deltaTime);

        //setters
        void setPGain(int32_t P);
        void setIGain(int32_t I);
        void setIBound(int32_t B);
};




#endif