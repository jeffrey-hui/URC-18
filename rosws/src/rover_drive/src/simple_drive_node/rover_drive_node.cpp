//
// Created by matthew on 21/08/17.
//

#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include "constants.h"
#include "ard_device.h"

rover_drive::ARDevice *dev;

void leftCallback(const std_msgs::Float32ConstPtr &msg) {
    int motorValue = static_cast<int>((std::min(std::max(-1.0f, msg->data), 1.0f) * rover_drive::MOTOR_OFFSET) + rover_drive::MOTOR_MID);
    for (uint8_t channel : rover_drive::LEFT_WHEELS) {
        dev->writeMicroseconds(channel, static_cast<uint16_t>(motorValue));
    }
}

void rightCallback(const std_msgs::Float32ConstPtr &msg) {
    int motorValue = static_cast<int>((std::min(std::max(-1.0f, msg->data), 1.0f) * rover_drive::MOTOR_OFFSET) + rover_drive::MOTOR_MID);
    for (uint8_t channel : rover_drive::RIGHT_WHEELS) {
        dev->writeMicroseconds(channel, static_cast<uint16_t>(motorValue));
    }
}


int main(int argc, char **argv) {
    ros::init(argc, argv, "rover_drive_node");
    ros::NodeHandle nh_;

    ROS_INFO_STREAM("Starting rover_drive_node...");
    int address, bus;
    nh_.param("address", address, 0x30);
    nh_.param("bus", bus, 1);
    ROS_INFO_STREAM("Opening arduino on bus " << bus << " address 0x" << std::hex << address);
    try {
        dev = new rover_drive::ARDevice(static_cast<uint8_t>(bus), static_cast<uint8_t>(address));
    }
    catch (std::runtime_error error) {
        ROS_FATAL_STREAM("Failed to open arduino: " << error.what());
        exit(1);
    }

    ros::Subscriber s1 = nh_.subscribe("/drive/left", 100, leftCallback);
    ros::Subscriber s2 = nh_.subscribe("/drive/right", 100, rightCallback);
    ROS_INFO_STREAM("Opening pins on the arduino");
    for (uint8_t pin : rover_drive::LEFT_WHEELS) {
        dev->openPin(pin);
    }
    for (uint8_t pin : rover_drive::RIGHT_WHEELS) {
        dev->openPin(pin);
    }
    ROS_INFO_STREAM("Opened arduino successfully!");
    ros::spin();

}
