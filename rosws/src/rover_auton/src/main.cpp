//
// Created by hamza on 11/05/18.
//


#include"rover.h"

void GPScallBack(const sensor_msgs::NavSatFix::ConstPtr& msg)
{
    GPSData.latitude = msg->latitude;
    GPSData.longitude = msg->longitude;
    GPSData.altitude = msg->altitude;
    ROS_INFO("I heard some shit");
}
void magCallBack(const sensor_msgs::MagneticField::ConstPtr& msg)
{
    magData.magnetic_field = msg->magnetic_field;
    ROS_INFO("I heard mag");
}
void execute(const rover_navigation::GotoPointGoalConstPtr& path_,actionlib::SimpleActionServer<rover_navigation::GotoPointAction>* as_){
    //sauto lol =
}

int main(int argc, char** argv){
    ros::init(argc,argv,"rover");
    rover r;

    ros::NodeHandle nh;
    ros::Publisher rover_pub = nh.advertise<geometry_msgs::Twist>("/wheel_diff_drive_controller/cmd_vel", 100);
    ros::Subscriber gps_sub = nh.subscribe("/gps/fix", 10, GPScallBack);
    ros::Subscriber mag_sub = nh.subscribe("/imu/mag", 10, magCallBack);
    ros::Rate loop_rate(10);
    geometry_msgs::Twist pwr_val;
    tf::TransformListener listener;
    actionlib::SimpleActionServer<rover_navigation::GotoPointAction> as_(nh,"tertiary_auton_server",boost::bind(&execute,_1,&as_),false);
    std::string action_name_;



    LinPID.init(ros::NodeHandle("~/linear"));
    AngPID.init(ros::NodeHandle("~/angular"));

    while (ros::ok())
    {
        current_time = ros::Time::now();
        pwr_val.linear.x = r.calculateDrivePower();
        pwr_val.angular.z = r.calculateTurnPower();
        rover_pub.publish(pwr_val);
        ros::spinOnce();
        last_time = current_time;
        loop_rate.sleep();
    }
}