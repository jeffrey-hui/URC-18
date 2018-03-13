#include <iostream>
#include <cmath.h>
#include "PID.h"
#include <ros/ros.h>
#include "std_msgs/String.h"
#include <tf/transform_listener.h>



using namespace std;

int main(int argc, char **argv)
{

  double pos_PID, dir_PID, rv_PID;
  ros::init(argc, argv, "tf_listener");
  ros::NodeHandle node;
  
  tf::TransformListener listener;
//lmaoooooooooooooooooooooooooooooooooooooooooooooooooo

  try{
    listener.lookupTransform("odom",ros::Time(0),transform);
  }
//lmaoooooooooooooooooooooooooooooooooooooooooooo
  ros::Subscriber sub = n.subscribe("odom",1000,odom);

  pos_PID = PID(0,0,0,0,0,0,0,0);
  dir_PID = PID(0,0,0,0,0,0,0,0); 
  rv_PID = bear_PID + pos_PID;

  //push rv_PID to the Arduino for power output
  //TODO: Add feedback loop for velocity with encoders 

  ros::spin();
  return 0;
}


//