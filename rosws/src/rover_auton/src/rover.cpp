//
// Created by hamza on 11/05/18.
//

#include "rover.h"
std::string receiveGPS;
ros::Time last_time, current_time;
control_toolbox::Pid LinPID;
control_toolbox::Pid AngPID;
rover::rover(){
}
rover::~rover(){}
double rover::getAngCoordinates(){
    return 1;

}
double rover::getLinCoordinates(){
    return 1;

}
double rover::calculateDrivePower(){
    double errorLin  = targetLin - rover::getLinCoordinates();
    return LinPID.computeCommand(errorLin,current_time-last_time);
}
double rover::calculateTurnPower(){
    double errorAng  = targetAng - rover::getAngCoordinates();
    return AngPID.computeCommand(errorAng,current_time-last_time);
}
