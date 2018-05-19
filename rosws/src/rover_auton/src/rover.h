//
// Created by hamza on 11/05/18.
//

#ifndef PROJECT_ROVER_H
#define PROJECT_ROVER_H
#include<ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/MagneticField.h>
#include <sensor_msgs/NavSatFix.h>
#include <control_toolbox/pid.h>
#include <tf/transform_listener.h>
#include<string>
using namespace control_toolbox;

extern control_toolbox::Pid LinPID;
extern control_toolbox::Pid AngPID;
extern control_toolbox::Pid MagPID;
extern ros::Time last_time, current_time;
extern sensor_msgs::NavSatFix GPSData;
extern sensor_msgs::MagneticField magData;
class rover {
private:
    double targetLat = 500;
    double targetLong = 200;
    double targetAng = 5;//just initialize for now
    double targetLin = 500;
    double speedLimit = 100;
public:
    rover();//constructor
    ~rover();
    double getAngCoordinates();//figure out how gps works later
    double getLinCoordinates();
    double calculateDrivePower();
    double calculateTurnPower();
    double getMagAngle();
    void calculateTargetAng();
    void calculateTargetLin();
};
#endif //PROJECT_ROVER_H
