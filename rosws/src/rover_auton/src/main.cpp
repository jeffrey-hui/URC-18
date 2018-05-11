//
// Created by hamza on 11/05/18.
//



#include"rover.h"

int main(int argc, char** argv){
    std::string sensor = "pub_name";//insert later when gabe gives me stuff
    ros::init(argc,argv,"rover");
    rover r;
    ros::NodeHandle nh;
    ros::Publisher rover_pub = nh.advertise<geometry_msgs::Twist>("/wheel_diff_drive_controller/cmd_vel", 100);
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