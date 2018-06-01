//
// Created by matthew on 5/20/18.
//

#include <ros/ros.h>
#include "nav_action_server.h"

int main(int argc, char ** argv) {
    ros::init(argc, argv, "navigator");

    rover_navigation::NavActionServer nas;
    ros::spin();
    return 0;
}