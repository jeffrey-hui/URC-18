//
// Created by hamza on 11/05/18.
//

#ifndef PROJECT_ROVER_H
#define PROJECT_ROVER_H
#include<ros/ros.h>
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"
#include "control_toolbox/pid.h"
#include<string>
using namespace control_toolbox;
extern std::string receiveGPS;
extern control_toolbox::Pid LinPID;
extern control_toolbox::Pid AngPID;
extern ros::Time last_time, current_time;
class rover {
private:
    double targetAng = 5;//just initialize for now
    double targetLin = 0;
public:
    rover();//constructor
    ~rover();
    double getAngCoordinates();//figure out how gps works later
    double getLinCoordinates();
    double calculateDrivePower();
    double calculateTurnPower();
};
#endif //PROJECT_ROVER_H
