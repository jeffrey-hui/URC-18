//
// Created by hamza on 11/05/18.
//

#include "rover.h"
ros::Time last_time, current_time;
control_toolbox::Pid LinPID;
control_toolbox::Pid AngPID;
sensor_msgs::NavSatFix GPSData;
sensor_msgs::MagneticField magData;
rover::rover(){
}
rover::~rover(){}
double rover::getAngCoordinates(){
    double angle  = atan2(GPSData.longitude,GPSData.latitude);
    return angle;
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
    return LinPID.computeCommand(errorLin,current_time-last_time);
}
double rover::calculateTurnPower(){
    calculateTargetAng();
    double errorAng  = targetAng - rover::getAngCoordinates();
    return AngPID.computeCommand(errorAng,current_time-last_time);
}
