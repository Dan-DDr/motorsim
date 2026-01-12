#include <inverter.h>

/**
 * @brief 
 * 
 * @param pwm (ah, al, bh, bl, ch, cl)
 * @return Vector3d (an, bn, cn)
 */
Vector3d Inverter::runModel(Vector<bool, 6> pwm) {
    Vector3d abc;

    abc(0) = vDC/2*(pwm(0) - pwm(1));
    abc(1) = vDC/2*(pwm(3) - pwm(2));
    abc(2) = vDC/2*(pwm(5) - pwm(4));

    return abc;
}