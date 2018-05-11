//
// Created by hamza on 11/05/18.
//

#include "rover.h"
rover::rover(){
    ros::NodeHandle n;
    ros::Subscriber sub = n.subscribe("test", 1000, &rover::GPScallBack,this);
}
rover::~rover(){}
double rover::getXCoordinates(){

}
double rover::getYCoordinates(){

}
double rover::calculateRightDrivePower(){

}
double rover::calculateLeftDrivePower(){

}
std::string rover::publishedValue(){
    //combine right and left drive into a string
    return "dummy";
}


void rover::GPScallBack(const std_msgs::String::ConstPtr& msg){
    receiveGPS = msg->data.c_str();
}