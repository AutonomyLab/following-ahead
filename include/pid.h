#ifndef _PID_H_
#define _PID_H_

class PIDImpl;
class PID
{
    public:
        // Kp -  proportional gain
        // Ki -  Integral gain
        // Kd -  derivative gain
        // max - maximum value of manipulated variable
        // min - minimum value of manipulated variable
        PID() {}
        void set( double max, double min, double Kp, double Kd, double Ki );
        PID( double max, double min, double Kp, double Kd, double Ki );

        // Returns the manipulated variable given a setpoint and current process value
        double calculate( double setpoint, double pv, double dt );
        ~PID();

    private:
        PIDImpl *pimpl;
};

#endif