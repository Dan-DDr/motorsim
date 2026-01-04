#include <iostream>
#include <Eigen/Dense>
#include <motorModel.h>
#include <controller.h>
#include <cstdlib>

using Eigen::Matrix3d;
using Eigen::Vector3d;

double dt = 0;
int steps = 0;

int runDQSimulation();


int main(int argc, char* argv[]) {

    if (argc >= 2) {
        steps = std::atoi(argv[1]);
        dt = std::atof(argv[2]);
    } else {
        exit(1);
    }

    if (dt == 0 || steps == 0) {
        exit(1);
    }

    runDQSimulation();
    
    return 0;
}


int runDQSimulation() {
    const int vdc = 40;


    Vector2d vdqHistory[steps] = {Vector2d::Zero()};
    Vector2d vdq;

    Vector<double, 5> x, xnew;


    x = Vector<double, 5>::Zero();

    Vector2d setpointHistory[steps] = {Vector2d::Zero()};
    Vector2d setpoint;

    
    MotorModel m(4, 0.125, 0.393e-3, 0.393e-3, 0, 0.296/4, 0.33e-3, 0, 0, 100);
    
    PID dCtrl(dt * 100, 0.125, 0.0, 0.0, vdc, -vdc);
    PID qCtrl(dt * 100, 0.125, 0.0, 0.0, vdc, -vdc);
    
    Encoder enc(14, 1);
    
    setpoint << 5, 0;
    double setpoint_start = 0.1;

    m.initHistory();


    for (int i = 0; i < steps; i++) {

        if (dCtrl.runModel(m.get_t(i)) && qCtrl.runModel(m.get_t(i)) && m.get_t(i) >= setpoint_start) {
            vdq(0) = dCtrl.calcOutput(setpoint(0) - static_cast<double>(m.get_x(i)(0)));
            vdq(1) = qCtrl.calcOutput(setpoint(1) - static_cast<double>(m.get_x(i)(1)));

        }
        m.runModelDqHistory(vdq, dt);
        vdqHistory[i] = vdq;
    }

    m.toCSV("output.csv");


    // Write to controller file
    std::ofstream file;
    file.open("main.csv");
    file << "v(0), v(1), setpoint(0), setpoint(1)" << std::endl;
    for (int i = 0; i < steps; i++) {
        auto v = vdqHistory[i];
        file << v(0) << ", " << v(1) << ", "<< setpoint(0) << ", " << setpoint(1)
        << ", " << enc.getAngle(m.get_x(i)(4)) << std::endl;
    }
    file.close();

    return 0;
}