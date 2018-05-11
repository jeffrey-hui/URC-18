//
// Created by hamza on 11/05/18.
//



#include"rover.h"

void GPScallBack(const std_msgs::String::ConstPtr& msg)
{
    ROS_INFO("I heard: [%s]", msg->data.c_str());
    receiveGPS = msg->data.c_str();
}

int main(int argc, char** argv){
    ros::init(argc,argv,"rover");
    rover r;
    ros::NodeHandle nh;
    ros::Publisher rover_pub = nh.advertise<geometry_msgs::Twist>("/wheel_diff_drive_controller/cmd_vel", 100);
    ros::Subscriber sub_test = nh.subscribe("chatter", 1000, GPScallBack);
    ros::Rate loop_rate(10);
    geometry_msgs::Twist pwr_val;
    while (ros::ok())
    {
        pwr_val.linear.x = r.calculateDrivePower();
        pwr_val.angular.z = r.calculateTurnPower();
        rover_pub.publish(pwr_val);
        ros::spinOnce();
        loop_rate.sleep();
    }
}