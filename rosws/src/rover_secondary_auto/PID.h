#ifndef PID_H
#define PID_H

class PID_imp;
class PID
{
    public:
		PID_imp(double dt, double max, double min, double kp, double ki, double kd);
		double calculate( double setpoint, double pv );
    private:
        PID_imp;
};
#endif
