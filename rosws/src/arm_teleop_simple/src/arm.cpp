//
// Created by hamza on 23/05/18.
//

#include <std_msgs/Float64.h>
#include <sensor_msgs/Joy.h>
#include<ros/ros.h>
void joyCallBack(const sensor_msgs::Joy::ConstPtr& joy){
    double z = joy->axes[1]*1000;
    double rawvel_outer = -joy->axes[2] * 20;

    double rawvel_inner = 0;
    if (joy->buttons[4])
        rawvel_inner -= 30;
    if (joy->buttons[5])
        rawvel_inner += 30;
    double rawvel_tilt = -joy->axes[5] * 300;
    std_msgs::Float64 data;
    //data = [0, rawvel_outer, rawvel_inner, rawvel_tilt];
    arm_pub.publish(data);

}
std_msgs::Float64 data;
ros::Publisher arm_pub;
int main(int argc, char** argv){
    ros::init(argc,argv,"arm_teleop");
    ros::NodeHandle nh;
    arm_pub = nh.advertise<std_msgs::Float64>("/arm_manual_controller/command", 10);
    ros::Subscriber joy_sub = nh.subscribe("joy", 10, joyCallBack);
    ros::Rate loop_rate(10);
    ros::spin();
}