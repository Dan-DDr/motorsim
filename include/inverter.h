#include <Eigen/Dense>

using namespace Eigen;
class Inverter {
    private:
        double vDC;
        bool ah, al, bh, bl, ch, cl;

    public:
        Vector3d runModel(Vector<double, 6>);
}