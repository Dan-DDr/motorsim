#ifndef LOGGING_H
#define LOGGING_H

#include <iostream>
#include <fstream>
#include <string>
#include <vector>

#include <motorModel.h>
#include <controller.h>
  
class Logger {
    public:
        int steps;

        std::vector<VectorXd> x;
        std::vector<double> t;
        
        void toCSV(std::string filename);
        void initHistory();
        void log(double input[]);

};

class MotorLogger : public Logger {
    private:
        MotorModel* mtr;
    public:
        MotorLogger(MotorModel* mtr) {
            MotorLogger::mtr = mtr;
        };
        void dump() {
            for (int i = 0; i < x.size(); i++) {
                    std::cout << x.at(i) << std::endl;
            }
        }

        void toCSV(std::string filename) {
            std::ofstream file;
            Vector<double, 5> x0;
            file.open(filename);
            file << "t, x(0), x(1), x(2), x(3), x(4)" << std::endl;
                for (int i = 0; i < x.size(); i++) {
                    x0 = x.at(i);
                    file << t.at(i) << ", " << x0(0) 
                    << ", " << x0(1) << ", " << x0(2) << ", " << x0(3) << ", "
                    << x0(4) << std::endl;
                }

                file.close();
        }

        void initHistory() {
            double t0 = 0;
            Vector<double, 5> x0 = Vector<double, 5>::Zero();
            MotorLogger::t.push_back(t0);
            MotorLogger::x.push_back(x0);
        }

        void log(double input[]) {
            Vector<double, 5> xnew, x0;
            double t0;

            t0 = t.back();
            x0 = x.back();

            Vector<double, 2> vDq;
            vDq(0) = input[0];
            vDq(1) = input[1];

            xnew = mtr->runModelDq(t0, vDq, x0, mtr->getTs());
            
            MotorLogger::t.push_back(t0 +  mtr->getTs());
            MotorLogger::x.push_back(xnew);
            // std::cout << x.back() << std::endl;

        }
};

class CtrlLogger : public Logger {
        private:
            PID* ctrl;
        
        public:

        CtrlLogger(PID* ctrl) {
            CtrlLogger::ctrl =  ctrl;
        };

        void dump() {
            for (int i = 0; i < x.size(); i++) {
                    std::cout << x.at(i) << std::endl;
            }
        }

        void toCSV(std::string filename) {
            std::ofstream file;
            Vector<double, 2> x0;
            file.open(filename);
            file << "t, error, output" << std::endl;
                for (int i = 0; i < x.size(); i++) {
                    x0 = x.at(i);
                    file << t.at(i) << ", " << x0(0) 
                    << ", " << x0(1) << std::endl;
                }
            file.close();
        }

        void initHistory() {
            double t0 = 0;
            Vector<double, 2> x0 = Vector<double, 2>::Zero();
            t.push_back(t0);
            x.push_back(x0);
        }

        void log(double input[]) {
            Vector<double, 2> xnew, x0;
            double t0;

            t0 = t.back();
            x0 = x.back();

            xnew(0) = *input;
            xnew(1) = ctrl->calcOutput(*input);
            
            t.push_back(t0 +  ctrl->getTs());
            x.push_back(xnew);

        }
};

#endif