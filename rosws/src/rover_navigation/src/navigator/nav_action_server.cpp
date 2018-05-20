//
// Created by matthew on 5/20/18.
//

#include "nav_action_server.h"

rover_navigation::NavActionServer::NavActionServer() : as("navigator", boost::bind(&rover_navigation::NavActionServer::execute, this, _1), false) {
    ros::NodeHandle nh("~");

    this->global_frame = nh.param<std::string>("global_frame", "world");
    this->cmd_vel = nh.advertise<geometry_msgs::Twist>("cmd_vel", 10);
    this->odom_data = nh.subscribe("odom", 10, &NavActionServer::odomCallback, this);
    this->as.start();
}

void rover_navigation::NavActionServer::execute(const rover_navigation::GotoPointGoalConstPtr &goal) {
    RunFlag rf = RunFlag(&this->goal_is_active);

    if (as.isPreemptRequested()) {
        as.setPreempted();
        return;
    }
    ros::Duration d = ros::Duration(0.05);
    ros::Time now = ros::Time::now();

    for (auto e : goal->path.points) {
        if (as.isPreemptRequested()) {
            as.setPreempted();
            return;
        }

        geometry_msgs::PointStamped ps;
        tfListener.transformPoint(this->global_frame, e, ps);

        this->nav.setGoalPosition(ps.point.x, ps.point.y);
        while (this->nav.getNavState() == GOING) {
            d.sleep();
            this->nav.update(ros::Time::now() - now);
            now = ros::Time::now();
            this->publishCommand();
        }
        this->publishCommand();
        this->publishCommand();
        this->nav.acceptGoal();
    }

    this->as.setSucceeded();
}

void rover_navigation::NavActionServer::odomCallback(const nav_msgs::OdometryConstPtr &odom) {
    if (this->goal_is_active) {
        geometry_msgs::PoseStamped ps;
        geometry_msgs::PoseStamped pi;

        pi.header = odom->header;
        pi.header.frame_id = odom->child_frame_id;
        pi.pose = odom->pose.pose;

        tfListener.transformPose(this->global_frame, pi, ps);
        tf::Quaternion q = tf::Quaternion(ps.pose.orientation.x, ps.pose.orientation.y, ps.pose.orientation.z, ps.pose.orientation.w);
        double r, p, y;
        tf::Matrix3x3(q).getRPY(r, p, y);

        this->nav.setCurrentPosition(ps.pose.position.x, ps.pose.position.y, y);
    }
}

void rover_navigation::NavActionServer::publishCommand() {
    geometry_msgs::Twist t;
    t.angular.z = this->nav.getCurrentAngular();
    t.linear.x  = this->nav.getCurrentLinear();
    this->cmd_vel.publish(t);
}
