//
// Created by hamza on 11/05/18.
//

#include "rover.h"
ros::Time last_time, current_time;
control_toolbox::Pid LinPID;
control_toolbox::Pid AngPID;
control_toolbox::Pid MagPID;
sensor_msgs::NavSatFix GPSData;
sensor_msgs::MagneticField magData;
rover::rover(){
}
rover::~rover(){}
double rover::getAngCoordinates(){
    double angle  = atan2(GPSData.longitude,GPSData.latitude);
    return angle;
}
double rover::getMagAngle(){
    double relative_angle = atan2(magData.magnetic_field.y,magData.magnetic_field.x);
    return relative_angle;
}
double rover::getLinCoordinates(){
    return sqrt((GPSData.longitude)*(GPSData.longitude)+(GPSData.latitude)*(GPSData.latitude));
}

void rover::calculateTargetAng(){
    targetAng = atan2(targetLong,targetLat);
}
void rover::calculateTargetLin(){
    targetAng = sqrt(targetLat*targetLat+targetLong*targetLong);
}
double rover::calculateDrivePower(){
    calculateTargetLin();
    double errorLin  = targetLin - rover::getLinCoordinates();
    double returnedValue  = LinPID.computeCommand(errorLin,current_time-last_time);
//    if(returnedValue>speedLimit)
//        returnedValue = speedLimit;
    return returnedValue;
}
double rover::calculateTurnPower(){
    calculateTargetAng();
    double errorAng  = targetAng - /*rover::getAngCoordinates()*/ rover::getMagAngle();
    return AngPID.computeCommand(errorAng,current_time-last_time);
}