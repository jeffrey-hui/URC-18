//
// Created by hamza on 23/05/18.
//

#include <std_msgs/Float64MultiArray.h>
#include <sensor_msgs/Joy.h>
#include<ros/ros.h>
void joyCallBack(const sensor_msgs::Joy::ConstPtr& joy){
    bool dummy_bool;
    double z = joy->axes[1]*1000;
    double rawvel_outer = -joy->axes[2] * 20;

    double rawvel_inner = 0;
    if (joy->buttons[4])
        rawvel_inner -= 30;
    if (joy->buttons[5])
        rawvel_inner += 30;
    double rawvel_tilt = -joy->axes[5] * 300;
    std_msgs::Float64MultiArray data;
    data.data[0] = 0;
    data.data[1] = rawvel_outer;
    data.data[2] = rawvel_inner;
    data.data[3] = rawvel_tilt;
    ROS_INFO("I heard something");
    //data = [0, rawvel_outer, rawvel_inner, rawvel_tilt];
    dummy_bool = true;

}
std_msgs::Float64MultiArray data;
ros::Publisher arm_pub;
extern bool dummy_bool;
int main(int argc, char** argv){
    ros::init(argc,argv,"arm_teleop");
    ros::NodeHandle nh;
    arm_pub = nh.advertise<std_msgs::Float64MultiArray>("/arm_manual_controller/command", 10);
    ros::Subscriber joy_sub = nh.subscribe("joy", 10, joyCallBack);
    ros::Rate loop_rate(10);
    bool dummy_bool;
    while (ros::ok())
    {
        if(dummy_bool == true)
            arm_pub.publish(data);
        dummy_bool = false;
        ros::spinOnce();
        loop_rate.sleep();
    }
}