//
// Created by matthew on 5/30/18.
//

#include <ros/ros.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Float32.h>
#include "oxyarm_dev.h"

rover_science::OxyArm *ultra_strength;

void handle_command(const std_msgs::Int16ConstPtr &m) {
    ultra_strength->writeMotor(m->data);
}

int main(int argc, char ** argv) {
    ros::init(argc, argv, "oxyarm");
    rover_science::OxyArm oxy_arm;
    ultra_strength = &oxy_arm;

    ROS_INFO_STREAM("Started oxy_arm");
    ros::NodeHandle nh = ros::NodeHandle();
    ros::Publisher pub = nh.advertise<std_msgs::Float32>("/science/oxy_sensor", 10);
    ros::Subscriber sub = nh.subscribe<std_msgs::Int16>("/arm_science_sensors/command", 10, &handle_command);

    ros::Rate rate(3.5);
    std_msgs::Float32 m;
    while (nh.ok()) {
        rate.sleep();
        m.data = oxy_arm.readOxy();
        pub.publish(
                m
        );
    }

    return 0;
}