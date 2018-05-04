#include "pid.h"
#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <stdlib.h>
#include "nav_msgs/odom"
#include "/wheel_diff_drive_controller/cmd_vel"

class roverPID{
private:
    ros::NodeHandle nh_; //represents the node... use this to sub and pub
    ros::Publisher cmd_vel_pub_;
    ros::Subscriber odom_info;
public:
    roverPID(ros::NodeHandle &nh)
    {
        nh_ = nh;
        cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/wheel_diff_drive_controller/cmd_vel",1);
        odom_info = nh_.subscribe("odom",10,PIDcontrol);

    }
    bool PIDcontrol(const nav_msgs::Odometry::ConstPtr& msg)
    {
        geometry_msgs::Twist base_cmd;
        double interval = 0.07; //PID base time interval

        //PID( dt,  max,  min,  Kp,  Kd,  Ki )
        PID angularPID = PID(interval, 0.4, 0, 1, 0.5, 0);
        PID linearPID = PID(interval, 0.6, 0, 1, 0.5, 1);
        //Target X and Y relative to current position.. could calculate this value from relative GPS coordinates... (Accept input from map?)
        double xCoo = 200;
        double yCoo = 100;
        // ROS_INFO("x: %f, y: %f",x,y); //ROS_INFO.. ROS_WARN...ROS_ERROR...ROS_FATAL
        
        double ang_Setpoint = tan(xCoo/yCoo); //theta
        double lin_Setpoint = sqrt(pow(xCoo,2)+ pow(yCoo,2));
        //Angular and distance setpoints using geometry


        //initial XY from ROS localization... somehow
        double linval = msg->pose.pose.position.x;
        double angval = msg->pose.pose.position.y;

        base_cmd.angular.z = angularPID.calculate(ang_Setpoint, angval);
        base_cmd.linear.x = linearPID.calculate(lin_Setpoint, linval)*cos(angval);
        cmd_vel_pub_.publish(base_cmd);
        return true;
    }

};

int main(argc, argv, char** argv){
  ros::init(argc,argv,"roverPID");
  ros::NodeHandle nh;
  roverPID driver(nh);
  ros::spin();
}
