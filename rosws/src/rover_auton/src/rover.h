//
// Created by hamza on 11/05/18.
//

#ifndef PROJECT_ROVER_H
#define PROJECT_ROVER_H
#include<ros/ros.h>
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"
#include<string>
#define Kp 1
class rover {
private:
    double targetAng = 5;//just initialize for now
    double targetLin = 0;
public:
    std::string receiveGPS;//this is a string because idk gps yet
    rover();//constructor
    ~rover();
    double getAngCoordinates();//figure out how gps works later
    double getLinCoordinates();
    void GPScallBack(const std_msgs::String::ConstPtr &msg);
    double calculateDrivePower();
    double calculateTurnPower();
};
#endif //PROJECT_ROVER_H
