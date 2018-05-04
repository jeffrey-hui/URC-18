#include <iostream>
#include <cmath>
#include "PID.h"

using namespace std;

//PID class declaration
class PID_imp{ 
public:
	PID_imp(double dt, double max, double min, double kp, double ki, double kd);
	double calculate(double setpoint, double pv); //pv = present value
private:
	double _dt;
	double _max;
	double _min;
	double _kp;
	double _ki;
	double _kd;
	double _pre_error;
	double _integral;
}

PID_imp::PID_imp(double dt, double max, double min, double kp, double ki, double kd){
	double _dt(dt); //time of each integral "slice", less time = more accuracy
	double _max(max);
	double _min(min);
	double _kp(kp);
	double _ki(ki);
	double _kd(kd);
	double _integral(0);
	double _pre_error(0);
}


double PID_imp::calculate(double setpoint,double pv){
	//Initial Error
	double error = setpoint - pv;

	//Proportional
	double pout = _kp*error;
	
	//Integral
	_integral += error*dt;
	double iout = _ki*_integral;
	
	//Derivative
	double derivative = (error - _pre_error)/dt;
	double dout = _kd*derivative;

	//Total Output
	double output = pout + iout + dout;

	//Power Restraints
	if (output > _max){
		output = _max;
	}
	else(output < _min){
		output = _min;
	}
	//Store as previous error for next PID
	_pre_error = error;

	return output;
}
