//
// Created by hamza on 01/06/18.
//



#include<ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float64.h>

int main(int argc, char** argv){
    ros::init(argc,argv,"rover");
    ros::NodeHandle nh;
    ros::Publisher rover_pub = nh.advertise<std_msgs::Float64>("/left_wheels_controller/cmd", 100);
    ros::Publisher rover_pub1 = nh.advertise<std_msgs::Float64>("/right_wheels_controller/cmd", 100);

    ros::Rate loop_rate(10);
   std_msgs::Float64 pwr_val;

    pwr_val.data = 1;
    int counter = 0;
    while(ros::ok()) {
        pwr_val.data = 1;
        rover_pub.publish(pwr_val);
        rover_pub1.publish(pwr_val);
        if (counter > 50)
            break;
        counter++;
        ros::spinOnce();
        loop_rate.sleep();
    }
    pwr_val.data = 0;
    rover_pub.publish(pwr_val);
    ros::spinOnce();
    ROS_INFO("Reached Target - VEX>>>>URC");

}