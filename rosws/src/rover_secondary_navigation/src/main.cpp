#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <stdlib.h>
#include <nav_msgs/Odometry.h>//TODO: FIX THIS
#include <control_toolbox/pid.h>
#include <algorithm>
#include <sensor_msgs/MagneticField.h>
#include <sensor_msgs/NavSatFix.h>



class roverPID{
private:
    ros::NodeHandle nh_;
    ros::Publisher cmd_vel_pub_;
    ros::Time currentTime;
    control_toolbox::Pid ang_pid;
    control_toolbox::Pid lin_pid;
    sensor_msgs::NavSatFix GPS;
    sensor_msgs::MagneticField Heading;

public:
    roverPID()
    {
        cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/wheel_diff_drive_controller/cmd_vel",1);
        ros::Subscriber odom_info = nh_.subscribe("odom",10, &roverPID::PIDcontrol, this);

        ang_pid.init(ros::NodeHandle("~/angle"));
        lin_pid.init(ros::NodeHandle("~/linear"));

        ros::param::set("/roverPID/angle", 10);
        ros::param::set("/roverPID/linear", 10);
        currentTime = ros::Time::now();
    }
    void PIDcontrol(const nav_msgs::OdometryConstPtr& msg)
    {
        geometry_msgs::Twist base_cmd;

        //Target Error
        double lin_Setpoint = sqrt((GPS.longitude)*(GPS.longitude)+(GPS.latitude)*(GPS.latitude));
        double ang_Setpoint = atan2(Heading.magnetic_field.y, Heading.magnetic_field.x); //theta
        double linval = pow((msg->pose.pose.position.x),2) + pow((msg->pose.pose.position.y),2); //has to be the distance to target aka error
        double angval = 0;

        //time interval with time
        ros::Duration a = ros::Time::now() - currentTime;
        currentTime = ros::Time::now();
        base_cmd.angular.z = ang_pid.computeCommand(angval - ang_Setpoint, a);
        base_cmd.linear.x = std::max(0.0, lin_pid.computeCommand(linval - lin_Setpoint, a)*cos(angval));
        cmd_vel_pub_.publish(base_cmd);
    }

};

int main(int argc, char** argv){
  ros::init(argc,argv,"roverPID");
  roverPID driver = roverPID();
  ros::spin();
}
