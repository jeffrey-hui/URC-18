//Reference Material: Ignore in CMAKE

#include "ros/ros.h"
#include "nav_msgs/odom"

void OdomCallback(const nav_msgs::Odometry::ConstPtr& msg){
	double x = msg->pose.pose.position.x;
	double y = msg->pose.pose.position.y;
	ROS_INFO("x: %f, y: %f",x,y); //ROS_INFO.. ROS_WARN...ROS_ERROR...ROS_FATAL
}

int main(int argc, char** argv){
	ros::init(argc,argv,"idk try");
	ros::NodeHandle nh;
	ros::Subsriber sub = nh.subscribe("odom",10,OdomCallback);
	ros::spin();
	return 0;
}
