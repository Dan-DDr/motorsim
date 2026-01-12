#ifndef MCB_H
#define MCB_H

#include <Eigen/Dense>
#include <cmath>

#define EIGEN_RUNTIME_NO_MALLOC

using namespace Eigen;

namespace MotorControlTools {

    /* Misc/Helper Functions */
    Matrix3d parkT(double angleRad);
    Matrix3d invParkT(double angleRad);

    Vector3d spaceVectorMod(Vector3d vAlphaBeta);
    
    class PWM {
        private:
            unsigned int counter;
            unsigned int clkFreq;
            unsigned int freqCnt;
            unsigned int MaxCount;
            unsigned int state;
            unsigned int cmpThres;
        public:
            PWM(unsigned int clkFreq, double frequency);
            int runModel(double t, double dutycycle);
    };
};

#endif