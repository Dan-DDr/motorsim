#ifndef INVERTER_H
#define INVERTER_H

#include <Eigen/Dense>

using namespace Eigen;
class Inverter {
    private:
        double vDC;
        bool ah, al, bh, bl, ch, cl;

    public:
        Inverter(double vDC) {
            Inverter::vDC = vDC;
        };
        Vector3d runModel(Vector<bool, 6> pwm);
};

#endif