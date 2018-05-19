//
// Created by hamza on 11/05/18.
//



#include"rover.h"
void GPScallBack(const sensor_msgs::NavSatFix::ConstPtr& msg)
{
    GPSData.latitude = msg->latitude;
    GPSData.longitude = msg->longitude;
    GPSData.altitude = msg->altitude;
}
void magCallBack(const sensor_msgs::MagneticField::ConstPtr& msg)
{
    magData.magnetic_field = msg->magnetic_field;
}

int main(int argc, char** argv){
    ros::init(argc,argv,"rover");
    rover r;

    ros::NodeHandle nh;
    ros::Publisher rover_pub = nh.advertise<geometry_msgs::Twist>("/wheel_diff_drive_controller/cmd_vel", 100);
    ros::Subscriber gps_sub = nh.subscribe("/gps/fix", 1000, GPScallBack);
    ros::Subscriber mag_sub = nh.subscribe("/imu/mag", 1000, magCallBack);
    ros::Rate loop_rate(10);
    geometry_msgs::Twist pwr_val;

    LinPID.initPid(27,1,2,0.3,-0,3);//change pid constants later
    AngPID.initPid(6,1,2,0.3,-0,3);

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