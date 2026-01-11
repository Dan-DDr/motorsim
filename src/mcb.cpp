#include <mcb.h>


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

Matrix3d MotorControlTools::invParkT(double angleRad) {
    Matrix3d mat;

    mat(0, 0) = std::cos(angleRad);
    mat(1, 0) = std::cos(angleRad - 2 * M_PI/3.0);
    mat(2, 0) = std::cos(angleRad + 2 * M_PI/3.0);

    mat(0, 1) = -1*std::sin(angleRad);
    mat(1, 1) = -1*std::sin(angleRad - 2 * M_PI/3.0);
    mat(2, 1) = -1*std::sin(angleRad + 2 * M_PI/3.0);

    mat(0, 2) = 1.0;
    mat(1, 2) = 1.0;
    mat(2, 2) = 1.0;

    return mat;
}

Vector3d MotorControlTools::spaceVectorMod(Vector3d vAlphaBeta) {
    double Ta, Tb, Tc;
    double a, b, c;

    a = vAlphaBeta(2);
    b = -1*vAlphaBeta(1) * std::sin(M_PI/3.0) - a;
    c = vAlphaBeta(1) * std::sin(M_PI/3.0) - a;


    unsigned int sector = 0;

    if (a > 0) {s}
}