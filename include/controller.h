#ifndef CONTROLLER_H
#define CONTROLLER_H

class PID {
    private:
        /* data */
        double kP, kI, kD;
        
        double Ts; // sample time

        double errorHistory[2];
        double outputHistory[2];

        double maxLimit;
        double minLimit;

        double tprev = 0;

        double clampOutput(double output);

        
    public:
        /* Constructors */
        PID(double Ts, double kP, double kI, double kD, double maxLimit, double minLimit);
        
        void reset();
        void setkP(double kP);
        void setkI(double kI);
        void setkD(double kD);
        double getTs() {return Ts;}

        bool runModel(double t) {
            if ((t  - tprev) > Ts) {
                tprev = t;
                return true;
            }
            return false;
        }

        double calcOutput(double error);
        double calcOutputForwardEuler(double error);
};

#endif
