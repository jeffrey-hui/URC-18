//
// Created by hamza on 11/05/18.
//

#include "rover.h"
std::string receiveGPS;
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
    return Kp*errorLin;
}
double rover::calculateTurnPower(){
    double errorAng  = targetAng - rover::getAngCoordinates();
    return Kp*errorAng;
}
