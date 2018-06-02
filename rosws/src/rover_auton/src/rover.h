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
#include <actionlib/server/simple_action_server.h>
#include <rover_navigation/GotoPointAction.h>
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
    double targetLat = 0;
    double targetLong = 0;
    double targetAng = 0;//just initialize for now
    double speedLimit = 1;
public:
    rover();//constructor
    ~rover();
    //figure out how gps works later
    double calculateDrivePower();
    double calculateTurnPower();
    double getMagAngle();
    void calculateTargetAng();
};
#endif //PROJECT_ROVER_H
