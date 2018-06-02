//
// Created by hamza on 01/06/18.
//



#include<ros/ros.h>
#include <geometry_msgs/Twist.h>


int main(int argc, char** argv){
    ros::init(argc,argv,"rover");
    ros::NodeHandle nh;
    ros::Publisher rover_pub = nh.advertise<geometry_msgs::Twist>("/wheel_diff_drive_controller/cmd_vel", 100);
    ros::Rate loop_rate(10);
    geometry_msgs::Twist pwr_val;

    pwr_val.linear.x = 1;
    pwr_val.angular.z = 0;
    int counter = 0;
    while(ros::ok()) {
        rover_pub.publish(pwr_val);
        pwr_val.linear.x = 1;
        rover_pub.publish(pwr_val);
        if (counter > 50)
            break;
        counter++;
        ros::spinOnce();
        loop_rate.sleep();
    }
    pwr_val.linear.x = 0;
    rover_pub.publish(pwr_val);
    ros::spinOnce();
    ROS_INFO("Reached Target - VEX>>>>URC");

}