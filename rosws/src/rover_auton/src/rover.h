//
// Created by hamza on 11/05/18.
//

#ifndef PROJECT_ROVER_H
#define PROJECT_ROVER_H
#include<ros/ros.h>
#include "std_msgs/String.h"
#include<string>
class rover {
public:
    std::string receiveGPS;//this is a string because idk gps yet
    rover();//constructor
    ~rover();
    double getXCoordinates();//figure out how gps works later
    double getYCoordinates();
    void GPScallBack(const std_msgs::String::ConstPtr &msg);
    double calculateRightDrivePower();
    double calculateLeftDrivePower();
    std::string publishedValue();//combines left and right drive into one message
};
#endif //PROJECT_ROVER_H
