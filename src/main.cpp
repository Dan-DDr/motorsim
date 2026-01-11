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
    Vector3d dummy;
    Vector3d alphabeta, dc;
    
    MotorModel m(4, 0.125, 0.393e-3, 0.393e-3, 0, 0.296/4, 0.33e-3, 0, 0, 100);
    
    PID dCtrl(dt * 100, 0.125, 0.0, 0.0, vdc, -vdc);
    PID qCtrl(dt * 100, 0.125, 0.0, 0.0, vdc, -vdc);
    
    MotorControlTools:: PWM p0(static_cast<unsigned int>(1/dt), 20000);
    MotorControlTools:: PWM p1(static_cast<unsigned int>(1/dt), 20000);
    MotorControlTools:: PWM p2(static_cast<unsigned int>(1/dt), 20000);


    Encoder enc(14, 1);
    
    setpoint << 0, 10;
    dummy << 0, 10, 0;
    double setpoint_start = 0.1;

    m.initHistory();


    for (int i = 0; i < steps; i++) {

        if (dCtrl.runModel(m.get_t(i)) && qCtrl.runModel(m.get_t(i)) && m.get_t(i) >= setpoint_start) {
            vdq(0) = dCtrl.calcOutput(setpoint(0) - static_cast<double>(m.get_x(i)(0)));
            vdq(1) = qCtrl.calcOutput(setpoint(1) - static_cast<double>(m.get_x(i)(1)));

        }
        alphabeta = MotorControlTools::invParkT(static_cast<double>(m.get_x(i)(4))) *  dummy;
        dc = (MotorControlTools::spaceVectorMod(alphabeta) / vdc)/2 + Vector3d(0.5, 0.5, 0.5);
        std::cout << p0.runModel(0, dc(0)) << ", " << p1.runModel(0, dc(1)) << ", "<< p2.runModel(0, dc(2)) << ", " << std::endl;
        m.runModelDqHistory(vdq, dt);
        vdqHistory[i] = vdq;
    }

    m.toCSV("output.csv");


    // // Write to controller file
    // std::ofstream file;
    // file.open("main.csv");
    // file << "v(0), v(1), setpoint(0), setpoint(1)" << std::endl;
    // for (int i = 0; i < steps; i++) {
    //     auto v = vdqHistory[i];
    //     file << v(0) << ", " << v(1) << ", "<< setpoint(0) << ", " << setpoint(1)
    //     << ", " << enc.getAngle(m.get_x(i)(4)) << std::endl;
    // }
    // file.close();

    return 0;
}