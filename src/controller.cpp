#include <controller.h>

/**
 * @brief Construct a new PID::PID object
 * 
 * @param Ts 
 * @param kP 
 * @param kI 
 * @param kD 
 * @param maxLimit 
 * @param minLimit 
 */
PID::PID(double Ts, double kP, double kI, double kD, double maxLimit, double minLimit) {
    
    PID::Ts = Ts;
    PID::kP = kP;
    PID::kI = kI;
    PID::kD = kD;
    PID::maxLimit = maxLimit;
    PID::minLimit = minLimit;
    
    reset();
}

/**
 * @brief 
 * 
 */
void PID::reset() {
    
    for (int i=0; i < 2; i++) {
        errorHistory[i] = 0;
    }

    for (int i=0; i < 2; i++) {
        outputHistory[i] = 0;
    }
}

/**
 * @brief 
 * 
 * @param output 
 * @return double 
 */
double PID::clampOutput(double output) {
    if (output >= maxLimit) {
        return maxLimit;
    } else if (output <= minLimit) {
        return minLimit;
    }
    return output;
}

// double PID::calcOutput(double error) {
//     double kPOutput, kDOutput, output;
    
//     kPOutput = (kP * error);
//     sumError += kI * error;

//     // clamp integrator error to prevent windup
//     sumError = clampOutput(sumError);


//     kDOutput =  kD * (error - prevError);
//     prevError = error;

//     output = kPOutput + sumError + kDOutput;

//     output = clampOutput(output);
//     return output;

// }

/**
 * @brief Calculates the output of the PID using Forward Euler approximation of
 * Integrator and Differentiator.
 * 
 * @param error Error into PID. Setpoint - Measured
 * @return double: Output of PID clamped between min and max limit.
 */
double PID::calcOutput(double error) {
    double k1, k2, k3;
    double output;

    k1 = kP +  kD;
    k2 = kP*Ts - 2*kP + kI*Ts - 2*kD;
    k3 = kP - kP*Ts + kI*Ts*Ts - kI*Ts + kD;

    output = k1 * error + k2 * errorHistory[0] + k3 * errorHistory[1] 
            - outputHistory[0] * (Ts - 2) - outputHistory[1]* (1 - Ts);

    output = clampOutput(output);

    outputHistory[1] = outputHistory[0];
    outputHistory[0] = output;

    errorHistory[1] = errorHistory[0];
    errorHistory[0] = error;

    return output;
}