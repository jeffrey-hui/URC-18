#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <stdlib.h>
#include <nav_msgs/Odometry.h>//TODO: FIX THIS
#include <control_toolbox/pid.h>
#include <algorithm>
#include <actionlib/server/simple_action_server.h>
#include <rover_secondary_navigation/FollowPathAction.h>

/*TODO:
 * Use tf2 to take path msg and extract x and y for 'xCoo' and 'yCoo'
 * implement actionlib
 *
 */
class roverPID{
private:
    ros::NodeHandle nh_;

    actionlib::SimpleActionServer<rover_secondary_navigation::FollowPathAction> as_;
    //create messages for the action server
    rover_secondary_navigation::FollowPathActionFeedback feedback_;
    rover_secondary_navigation::FollowPathActionResult result_;
    rover_secondary_navigation::FollowPathActionGoal goal_;

    ros::Publisher cmd_vel_pub_;
    ros::Subscriber odom_info;

    ros::Time currentTime;
    control_toolbox::Pid ang_pid;
    control_toolbox::Pid lin_pid;
public:
    roverPID()
    {
//        as_(nh_,name,boost::bind(&roverPID::PIDcontrol,this,_1), false), FollowPath_(name){
//            as_.start();
//        }
        cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/wheel_diff_drive_controller/cmd_vel",1);
        odom_info = nh_.subscribe("odom",10, &roverPID::PIDcontrol, this);

        ang_pid.init(ros::NodeHandle("~/angle"));
        lin_pid.init(ros::NodeHandle("~/linear"));
        currentTime = ros::Time::now();
    }
    void PIDcontrol(const nav_msgs::OdometryConstPtr& msg)
    {
        geometry_msgs::Twist base_cmd;

        //Target Error
        double xCoo = 0;
        double yCoo = 0;
        double ang_Setpoint = atan2(yCoo, xCoo); //theta
        double lin_Setpoint = sqrt(pow(xCoo,2)+ pow(yCoo,2));
        //Angular and distance setpoints using geometry

        //initial XY from ROS localization... somehow
        double linval = pow((msg->pose.pose.position.x),2) + pow((msg->pose.pose.position.y),2); //has to be the distance to target aka error
        double angval = 0; //TODO: Convert from pose orientation...?

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
