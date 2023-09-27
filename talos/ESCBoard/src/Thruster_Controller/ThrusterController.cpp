#include "ThrusterController.h"

//calc ff dshot value
int32_t ThrusterController::feedForward(int32_t targetRPM){
    //replace with cubic spline later
    int32_t ffControl = targetRPM * .001 * 1000000;

    //divide by 1000000 to account for scaled consts
    return ffControl / 1000000;
}

//call this each cycle
int32_t ThrusterController::control(int32_t targetRPM, int32_t currentRPM, int32_t deltaTime){
    //rpm error - this value should be practically less than 10,000
    int32_t currentError = targetRPM - currentRPM;

    //limit max error to ensure not overrunning int value 
    if(currentError  > 2100000000 / this->Pgain){
        currentError = 2100000000 / this->Pgain;
    } else if(currentError < -2100000000 / this->Pgain){
        currentError = -2100000000 / this->Pgain;
    }

    //calculate ff control
    int32_t ffControl = feedForward(targetRPM);

    //calculate p control - divide by 1000000 is scaling for const
    int32_t pControl = (currentError * this->Pgain) / 1000000;

    //calculate I 
    int32_t iControl = 0;

    if(currentError < this->Ibound && currentError > -this->Ibound){
        //only if within bound

        //add current to sum of error
        this->sumOfError += currentError * deltaTime;
        
        //check to ensure int value is not overrun - very possible when tuning gains 
        if(sumOfError  > 2100000000 / this->Igain){
            sumOfError = 2100000000 / this->Igain;
        } else if(sumOfError < -2100000000 / this->Igain){
            sumOfError = -2100000000 / this->Igain;
        }   
        
        //divide by 1000000 is scaling for const
        iControl = (this->sumOfError * Igain) / 1000000;
        
    }else{
        //reset error sum (I) as the control has slipped bound
        this->sumOfError = 0;
    }

    //calculate total control
    int32_t dShotControl = iControl + pControl + ffControl;

    //enforce hard limit
    if(dShotControl > this->hardLimit){
        dShotControl = this->hardLimit;
    } else if (dShotControl < -this->hardLimit){
        dShotControl = -this->hardLimit;
    }

    return dShotControl;
}

//setters
void ThrusterController::setPGain(int32_t P){
    this->Pgain = P;
}

void ThrusterController::setIGain(int32_t I){
    this->Igain = I;
}

void ThrusterController::setIBound(int32_t B){
    this->Ibound = B;
}