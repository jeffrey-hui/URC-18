//
// Created by matthew on 5/20/18.
//

#ifndef PROJECT_NAV_ACTION_SERVER_H
#define PROJECT_NAV_ACTION_SERVER_H

#include <rover_navigation/GotoPointAction.h>
#include <actionlib/server/simple_action_server.h>
#include <tf/transform_listener.h>
#include <nav_msgs/Odometry.h>
#include "navigator.h"

namespace rover_navigation {

    struct RunFlag {
        RunFlag(bool *b) : b(b){
            *b = true;
        };

        ~RunFlag() {
            *b = false;
        };


        bool *b;
    };

    class NavActionServer {
    public:
        NavActionServer();

        void execute(const rover_navigation::GotoPointGoalConstPtr &goal);
        void odomCallback(const nav_msgs::OdometryConstPtr &odom);

    private:
        void publishCommand();

        Navigator nav;
        bool goal_is_active;
        actionlib::SimpleActionServer<rover_navigation::GotoPointAction> as;

        std::string global_frame;

        ros::Subscriber odom_data;
        ros::Publisher cmd_vel;
        tf::TransformListener tfListener;
    };

}

#endif //PROJECT_NAV_ACTION_SERVER_H
