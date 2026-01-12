#ifndef MOTORMODEL_H
#define MOTORMODEL_H

#include <Eigen/Dense>
#include <iostream>
#include <vector>
#include <fstream>
#include <string>
#include <cmath>
#include <random>

#include <mcb.h>

using namespace Eigen;

#define EIGEN_RUNTIME_NO_MALLOC


double wrapAngle(double angleRad);

class MotorModel {
    private:

        double Rs, Ld, Lq, L0, pmFlux;
        double J, bLoad, tLoad, efficiency;
        int n;

        double Ts;

        std::vector<Vector<double, 5>> x;
        std::vector<double> t;


        /* Solver Functions */
        Vector<double, 5> odeStep(double t, Vector<double, 3> v, Vector<double, 5> x);
        Vector<double, 5> rk4Step(double t, Vector<double, 3> v, Vector<double, 5> x, double dt);
        Vector<double, 5> rk4StepDq(double t, Vector<double, 2> vDq, Vector<double, 5> x, double dt);
        Vector<double, 5> rk4StepAbc(double t, Vector<double, 3> vAbc, Vector<double, 5> x, double dt);
        
    public:
        

        std::vector<double> get_t()             { return t; }
        std::vector<Vector<double,5>> get_x()   { return x; }
        double get_t(int i)                     { return t.at(i); }
        Vector<double, 5> get_x(int i)          { return x.at(i); }
        double getTs() {return Ts;}

    
        /*  Constructors */
        MotorModel(double Ts, int n, double Rs, double Ld, double Lq, double L0, 
                    double pmFlux, double J, double bLoad, double tLoad, double efficiency);
        
        // ~MotorModel();
        
        void initHistory() {
            Vector<double, 5> x0;
            double t0;
            
            x0 = Vector<double, 5>::Zero();
            t0 = 0;

            t.push_back(t0);
            x.push_back(x0);
            
        };

        /* Run Model functions*/
        Vector<double, 5> runModelDq(double t, Vector2d vDq, Vector<double, 5> x0, double dt) {

            Vector<double, 5> xnew;
            xnew = rk4StepDq(t, vDq, x0, dt);
            return xnew;
        }

        Vector<double, 5> runModelAbc(double t, Vector3d vAbc, Vector<double, 5> x0, double dt) {

            Vector<double, 5> xnew;
            xnew = rk4StepAbc(t, vAbc, x0, dt);
            return xnew;
        }

        void runModelDqHistory(Vector2d vDq, double dt) {
            Vector<double, 5> xnew, x0;
            double t0;

            t0 = t.back();
            x0 = x.back();

            xnew = runModelDq(t0, vDq, x0, dt);
            
            t.push_back(t0 +  dt);
            x.push_back(xnew);

        }

        void runModelAbcHistory(Vector3d vAbc, double dt) {
            Vector<double, 5> xnew, x0;
            double t0;

            t0 = t.back();
            x0 = x.back();

            xnew = runModelAbc(t0, vAbc, x0, dt);
            
            t.push_back(t0 +  dt);
            x.push_back(xnew);

        }

        void toCSV(std::string filename);
};



class Encoder {
    private:
        double angleRad;
        unsigned int encoderOut;
        unsigned int numBits;
        
        double noiseFactor;
        unsigned int delaySamples;

        double getNoise();
    public:
        Encoder(int numBits, int noiseFactor);
        unsigned int getAngle(double angleRad);

};


#endif