//
// Created by matthew on 5/20/18.
//

#include "navigator.h"

rover_navigation::Navigator::Navigator() {
    ros::NodeHandle nh("~");

    this->max_speed = nh.param("max_speed", 2.0); // mps
    this->max_ang = nh.param("max_ang", 2.5);  // rps
    this->tolerance = nh.param("goal_tolerance", 1.0);

    this->ang.init(ros::NodeHandle("~/ang"), false);
    this->lin.init(ros::NodeHandle("~/lin"), false);

    this->navState = NavState::NOT_RUNNING;
}

void rover_navigation::Navigator::setGoalPosition(double x, double y) {
    this->goalX = x;
    this->goalY = y;
    this->navState = NavState::GOING;
}

void rover_navigation::Navigator::setCurrentPosition(double x, double y, double curAng) {
    this->curX = x;
    this->curY = y;
    this->curAng = curAng;
}

void rover_navigation::Navigator::update(ros::Duration dt) {
    double dist = std::sqrt(
            std::pow(goalX - curX, 2) + std::pow(goalY - curY, 2)
    );

    if (dist < tolerance) {
        ROS_INFO_STREAM("OK!");
        this->navState = GOAL_OK;
    }

    double angTarget = std::atan2(goalY - curY, goalX - curX);
    double angErr = angTarget - curAng;
    if (this->navState == GOING) {
        this->curAngC = std::max(-this->max_ang, std::min(this->max_ang, ang.computeCommand(angErr, dt)));
        this->curLin = std::min(this->max_speed, std::max(0.0d, lin.computeCommand(dist, dt) * std::cos(angErr)));
    }
    else {
        this->curAngC = this->curLin = 0.0d;
    }
}

void rover_navigation::Navigator::acceptGoal() {
    if (this->navState == GOAL_OK) {
        this->navState = NOT_RUNNING;
    }
}

double rover_navigation::Navigator::getCurrentLinear() {
    return this->curLin;
}

double rover_navigation::Navigator::getCurrentAngular() {
    return this->curAngC;
}

rover_navigation::NavState rover_navigation::Navigator::getNavState() {
    return this->navState;
}
