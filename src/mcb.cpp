#include <mcb.h>

/**
 * @brief 
 * 
 * @param angleRad 
 * @return Matrix3d 
 */
Matrix3d MotorControlTools::parkT(double angleRad) {
    Matrix3d mat;

    mat(0, 0) = std::cos(angleRad);
    mat(0, 1) = std::cos(angleRad - 2 * M_PI/3.0);
    mat(0, 2) = std::cos(angleRad + 2 * M_PI/3.0);

    mat(1, 0) = -1*std::sin(angleRad);
    mat(1, 1) = -1*std::sin(angleRad - 2 * M_PI/3.0);
    mat(1, 2) = -1*std::sin(angleRad + 2 * M_PI/3.0);

    mat(2, 0) = 0.5;
    mat(2, 1) = 0.5;
    mat(2, 2) = 0.5;

    return 2.0/3.0 * mat;

}

// Matrix3d MotorControlTools::invParkT(double angleRad) {
//     Matrix3d mat;

//     mat(0, 0) = std::cos(angleRad);
//     mat(1, 0) = std::cos(angleRad - 2 * M_PI/3.0);
//     mat(2, 0) = std::cos(angleRad + 2 * M_PI/3.0);

//     mat(0, 1) = -1*std::sin(angleRad);
//     mat(1, 1) = -1*std::sin(angleRad - 2 * M_PI/3.0);
//     mat(2, 1) = -1*std::sin(angleRad + 2 * M_PI/3.0);

//     mat(0, 2) = 1.0;
//     mat(1, 2) = 1.0;
//     mat(2, 2) = 1.0;

//     return mat;
// }

/**
 * @brief 
 * 
 * @param angleRad 
 * @return Matrix3d 
 */
Matrix3d MotorControlTools::invParkT(double angleRad) {
    Matrix3d mat;

    mat(0, 0) = std::cos(angleRad);
    mat(0, 1) = -1*std::sin(angleRad);
    mat(0, 2) = 0.0;

    mat(1, 0) = std::sin(angleRad);
    mat(1, 1) = std::cos(angleRad);
    mat(1, 2) = 0.0;
    
    mat(2, 0) = 0.0;
    mat(2, 1) = 0.0;
    mat(2, 2) = 1.0;


    return mat;
}

/**
 * @brief 
 * 
 * @param vAlphaBeta 
 * @return Vector3d 
 */
Vector3d MotorControlTools::spaceVectorMod(Vector3d vAlphaBeta) {
    double Ta, Tb, Tc;
    Vector3d dc;

    double a, b, c;

    a = vAlphaBeta(0);
    b = vAlphaBeta(1) * std::sin(M_PI/3.0) - 0.5*a;
    c = -1*vAlphaBeta(1) * std::sin(M_PI/3.0) - 0.5*a;

    double Vmax, Vmin, Vcom;

    if (a > b) {
        Vmax = a;
        Vmin = b;
    } else {
        Vmax = b;
        Vmin = a;
    }

    if (c > Vmax) {
        Vmax = c;
    } else if (c < Vmin) {
        Vmin = c;
    }

    Vcom = 0.5f * (Vmax + Vmin);

    dc(0) = a - Vcom;
    dc(1) = b - Vcom;
    dc(2) = c - Vcom;

    return dc;
}

/**
 * @brief Construct a new Motor Control Tools:: P W M:: P W M object
 * 
 * @param clkFreq 
 * @param frequency 
 */
MotorControlTools::PWM::PWM(unsigned int clkFreq, double frequency) {
    counter = 0;
    PWM::clkFreq = clkFreq;
    MaxCount = static_cast<unsigned int>(clkFreq / (2* frequency));
    state = 0;
}

/**
 * @brief 
 * 
 * @param t 
 * @param dutycycle 
 * @return int 
 */
int MotorControlTools::PWM::runModel(double t, double dutycycle) {
    if (state == 0)
        counter += 1;
    else if (state == 1)
        counter -= 1;

    if (counter >= MaxCount && state == 0) {
        state = 1;
    } else if (counter == 0 && state == 1) {
        state = 0;
    }
    
    if (counter == 0)
        cmpThres = static_cast<unsigned int>(dutycycle * MaxCount);

    if (cmpThres >= counter) {
        return 1;
    } else {
        return 0;
    }

}