//
// Created by hamza on 11/05/18.
//



#include"rover.h"
#include<ros/ros.h>
#include "std_msgs/String.h"
int main(int argc, char** argv){
    std::string sensor = "pub_name";//insert later when gabe gives me stuff
    ros::init(argc,argv,"rover");
    rover r;
    ros::NodeHandle nh;
    ros::Publisher rover_pub = nh.advertise<std_msgs::String>("powerValue", 100);
    ros::Rate loop_rate(10);
    std_msgs::String pwr_val;
    while (ros::ok())
    {
        pwr_val.data = r.publishedValue();
        rover_pub.publish(pwr_val);
        ros::spinOnce();
        loop_rate.sleep();
    }
}