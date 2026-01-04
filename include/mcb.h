#include <Eigen/Dense>
#include <cmath>

#define EIGEN_RUNTIME_NO_MALLOC

using namespace Eigen;

namespace MotorControlTools {

    /* Misc/Helper Functions */
    Matrix3d parkT(double angleRad);
    Matrix3d invParkT(double angleRad);

    Vector3d spaceVectorMod(Vector3d vAlphaBeta);

};