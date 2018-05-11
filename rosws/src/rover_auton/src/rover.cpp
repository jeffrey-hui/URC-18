//
// Created by hamza on 11/05/18.
//

#include "rover.h"
rover::rover(){
    ros::NodeHandle n;
    ros::Subscriber sub = n.subscribe("test", 1000, &rover::GPScallBack,this);
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
    return Kp*errorLin;
}
double rover::calculateTurnPower(){
    double errorAng  = targetAng - rover::getAngCoordinates();
    return Kp*errorAng;
}

void rover::GPScallBack(const std_msgs::String::ConstPtr& msg){
    receiveGPS = msg->data.c_str();
}