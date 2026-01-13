#include <iostream>
#include <Eigen/Dense>
#include <motorModel.h>
#include <logging.h>
#include <controller.h>
#include <cstdlib>

using Eigen::Matrix3d;
using Eigen::Vector3d;

double dt = 0;
int steps = 0;

int runDQSimulation();
int testLogger();


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

    // runDQSimulation();
    testLogger();
    
    return 0;
}

int testLogger() {
    const int vdc = 40;
    
    MotorModel m(dt, 4, 0.125, 0.393e-3, 0.393e-3, 0, 0.296/4, 0.33e-3, 0, 0, 100);
    
    PID dCtrl(dt * 10, 0.125, 0.5, 0.0, vdc, -vdc);
    PID qCtrl(dt * 10, 0.125, 0.5, 0.0, vdc, -vdc);

    MotorLogger motorLogger(&m);
    CtrlLogger qctrlLogger(&qCtrl);
    CtrlLogger dctrlLogger(&dCtrl);

    motorLogger.initHistory();
    qctrlLogger.initHistory();
    dctrlLogger.initHistory();

    double setpoint[2] = {10.0, 0.0};
    double error[2] = {0, 0};
    double vdq[2] = {0, 0};
    double setpoint_start = 0.1;

    motorLogger.dump();
    for (int i = 0; i < steps; i++) {
        if (dCtrl.runModel(motorLogger.t.at(i)) && 
            qCtrl.runModel(motorLogger.t.at(i)) && 
            (motorLogger.t.at(i) >= setpoint_start)) {

            error[0] = setpoint[0] - static_cast<double>(motorLogger.x.back()(0));
            error[1] = setpoint[1] - static_cast<double>(motorLogger.x.back()(1));
            
            std::cout << "error: "<< error[0] << ", " << error[1] << std::endl;

            dctrlLogger.log(&(error[0]));
            qctrlLogger.log(&(error[1]));
            
            vdq[0] = dctrlLogger.x.back()(1);
            vdq[1] = qctrlLogger.x.back()(1);

            std::cout << "vdq: "<< vdq[0] << ", " << vdq[1] << std::endl;
        }
        std::cout << "x0: "<< motorLogger.x.back()(0) << std::endl;
        motorLogger.log(vdq);

    }

    dctrlLogger.toCSV("dctrl.csv");
    qctrlLogger.toCSV("qctrl.csv");
    motorLogger.toCSV("motor.csv");

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
    
    MotorModel m(dt, 4, 0.125, 0.393e-3, 0.393e-3, 0, 0.296/4, 0.33e-3, 0, 0, 100);
    
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