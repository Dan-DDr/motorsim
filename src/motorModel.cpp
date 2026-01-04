#include <motorModel.h>

double wrapAngle(double angleRad) {
    double ang;
    ang = std::fmod(angleRad, 2 * M_PI);
    
    if (ang < 0) {
        ang = 2 * M_PI - ang;
    }
    
    return ang;
}

MotorModel::MotorModel(int n, double Rs, double Ld, double Lq, double L0, 
                    double pmFlux, double J, double bLoad, double tLoad, double efficiency) {
    MotorModel::n = n;
    MotorModel::Rs = Rs;
    MotorModel::Ld = Ld;
    MotorModel::Lq = Lq;
    MotorModel::L0 = L0;
    MotorModel::pmFlux = pmFlux;
    MotorModel::J = J;
    MotorModel::bLoad = bLoad;
    MotorModel::tLoad = tLoad;
    MotorModel::efficiency = efficiency;

}

Vector<double, 5> MotorModel::odeStep(double t, Vector<double, 3> v, Vector<double, 5> x) {
    Vector<double, 5> dxdt;
    double eTorque;
    
    dxdt(0) = 1/Ld * (v(0) - Rs*x(0) + x(3)*n*x(1)*Lq);
    dxdt(1) = 1/Ld * (v(1) - Rs*x(1) - x(3)*n*(x(0)*Ld + pmFlux));
    
    eTorque = 3/2 * n*(x(1)*(x(0)*Ld + pmFlux) - x(0)*x(1)*Lq);

    dxdt(3) = (eTorque - tLoad)/J - bLoad*x(3);
    dxdt(4) = x(3);

    if (L0 != 0) {
        dxdt(2) = 1/L0 * (v(2) - Rs*x(2));
    } else {
        dxdt(2) = 0;
    }
    return dxdt;
}

Vector<double, 5> MotorModel::rk4Step(double t, Vector<double, 3> v, Vector<double, 5> x, double dt) {
    Vector<double, 5> k1, k2, k3, k4, xnew;
    k1 = odeStep(t, v,  x);
    k2 = odeStep(t + dt/2, v, x + k1/2 * dt);
    k3 = odeStep(t + dt/2, v, x + k2/2 * dt);
    k4 = odeStep(t + dt, v, x + k3 * dt);

    xnew = x +  dt/6 * (k1 + 2*k2 + 2*k3 + k4);

    xnew(4) = wrapAngle(static_cast<double>(xnew(4)));
    return xnew;
}

Vector<double, 5> MotorModel::rk4StepDq(double t, Vector<double, 2> vDq, Vector<double, 5> x, double dt) {
    Vector3d v;
    Vector<double, 5> xnew;

    v(0) = vDq(0);
    v(1) = vDq(1);
    v(2) = 0;

    xnew = rk4Step(t, v, x, dt);
    return xnew;
}

Vector<double, 5> MotorModel::rk4StepAbc(double t, Vector<double, 3> vAbc, Vector<double, 5> x, double dt) {
    Vector3d v, i, vtmp;
    Vector<double, 5> xnew;

    v = MotorControlTools::parkT(static_cast<double>(x(4))) * vAbc;
    i = MotorControlTools::parkT(static_cast<double>(x(4))) * x(seq(0, 2));
    

    x(0) = i(0);
    x(1) = i(1);
    x(2) = i(2);
    
    xnew = rk4Step(t, v, x, dt);

    vtmp = MotorControlTools::invParkT(static_cast<double>(xnew(4))) * xnew(seq(0, 2));

    xnew(0) = vtmp(0);
    xnew(1) = vtmp(1);
    xnew(2) = vtmp(2);

    return xnew;
}


void MotorModel::toCSV(std::string filename) {

    std::ofstream file;
    Vector<double, 5> x0;
    file.open(filename);

    file << "t, x(0), x(1), x(2), x(3), x(4)" << std::endl;
    for (int i = 0; i < x.size(); i++) {
        x0 = get_x(i);

        file << get_t(i) << ", " << x0(0) 
        << ", " << x0(1) << ", " << x0(2) << ", " << x0(3) << ", "
        << x0(4) << std::endl;
    }

    file.close();
}

Encoder::Encoder(int numBits, int noiseFactor) {
    Encoder::numBits = numBits;
    Encoder::noiseFactor = noiseFactor;
}

double Encoder::getNoise() {
    std::random_device rd{};
    std::mt19937 gen{rd()};
    
    std::normal_distribution<double> dist(0, 1);

    return dist(gen);
}

unsigned int Encoder::getAngle(double angleRad) {
    double noise;
    
    noise = getNoise();

    angleRad = (noiseFactor * noise + angleRad);
    angleRad = wrapAngle(angleRad) * ((1 << numBits) - 1);
    angleRad = angleRad/(2 * M_PI);

    return static_cast<int>(angleRad);
}